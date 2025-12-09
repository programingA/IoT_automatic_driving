# line_rate_detect.py
# 카메라 영상 기반 차선 감지 + offset 계산 + ROI 시각화 + Arduino 시리얼 명령 전송

import cv2
import numpy as np
import serial
import time
import os  # 🔥 추가: DISPLAY 확인용

# 🔥 DISPLAY 환경변수가 있을 때만 GUI 사용
USE_GUI = bool(os.environ.get("DISPLAY"))


class LaneDetectorSW:
    def __init__(self,
                 serial_port="/dev/ttyACM0",
                 baud=9600,
                 use_internal_serial=True):
        # 슬라이딩 윈도우 파라미터
        self.n_windows = 9
        self.margin = 60
        self.minpix = 50

        # 🔧 이진화/노이즈 제거 튜닝 파라미터
        # HSV에서 흰색 범위 (필요하면 숫자만 바꿔서 실험)
        self.lower_white = np.array([0,   0, 185])   # V 최소 160 → 180 으로 올림
        self.upper_white = np.array([185, 70, 255])  # S 상한 60 → 40 으로 줄임 (더 “진짜 흰색”만)

        # 모폴로지 커널 크기
        self.kernel_size = 3    # 기존 5 → 3으로 줄여서 너무 두껍게 안 만들게
        # 작은 블랍 제거 기준 (픽셀 수)
        self.min_area = 2500    # 기존 1000 → 2500 정도로 상향

        # 🔥 학습된 차선 폭(px) 저장용 (양쪽 다 보일 때 업데이트)
        self.lane_width_px = None

        # 🔥 마지막으로 보낸 명령 & 선이 완전히 사라진 시점
        self.last_cmd = "F"
        self.lost_start_time = None  # 두 선 다 안 보이기 시작한 시간 (L/R 상태에서만 사용)

        # 내부 시리얼 자동 전송 사용 여부
        self.use_internal_serial = use_internal_serial

        # 시리얼 초기화 (Jetson ↔ Arduino)
        try:
            self.ser = serial.Serial(serial_port, baud, timeout=1)
            time.sleep(2)
            print(f"[INFO] Serial connected: {serial_port} @ {baud}")
        except Exception as e:
            print("[WARN] Serial 초기화 실패:", e)
            self.ser = None

    def color_binary(self, frame):
        """
        BGR 프레임 → 흰 차선만 남긴 바이너리 이미지 반환
        노이즈를 줄이기 위해:
        - 가우시안 블러
        - HSV 범위
        - (완화된) 적응형 이진화
        - 모폴로지 open + close
        - 연결요소의 테두리 모양(컨투어) 기반 노이즈 제거
          · 테두리가 울퉁불퉁하면 노이즈
          · 테두리가 깔끔하거나 직사각형/선형이면 차선 후보로 유지
        """
        # 1) 블러 + HSV 이진화
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask_white = cv2.inRange(hsv, self.lower_white, self.upper_white)

        # 2) 적응형 이진화 (이전보다 약하게)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        adaptive = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY,
            31,   # 블록 크기 ↑ → 더 부드럽게
            0     # C = 0 → 너무 많이 깎지 않음 (이전 -5보다 훨씬 약함)
        )
        # HSV 마스크와 결합
        mask_white = cv2.bitwise_and(mask_white, adaptive)

        # 3) 모폴로지 (OPEN 1회로 완화)
        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel, iterations=1)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel, iterations=2)

        # 이진화 확정
        _, binary = cv2.threshold(mask_white, 50, 255, cv2.THRESH_BINARY)

        # 4) 연결요소 분석 + 테두리(컨투어) 기반 노이즈 필터
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary)
        clean = np.zeros_like(binary)

        h_img, w_img = binary.shape[:2]

        for i in range(1, num_labels):
            x = stats[i, cv2.CC_STAT_LEFT]
            y = stats[i, cv2.CC_STAT_TOP]
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            area = stats[i, cv2.CC_STAT_AREA]

            # 1차: 너무 작은 blob 제거 (기존 유지)
            if area < self.min_area:
                continue

            # 해당 라벨만 마스크로 추출
            comp_mask = np.zeros_like(binary)
            comp_mask[labels == i] = 255

            contours, _ = cv2.findContours(
                comp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue

            cnt = max(contours, key=cv2.contourArea)
            peri = cv2.arcLength(cnt, True)
            if peri < 1:
                continue

            contour_area = cv2.contourArea(cnt)
            if contour_area < self.min_area * 0.5:
                # 컨투어 기준 면적이 너무 작으면 패스
                continue

            # ▸ 테두리의 "울퉁불퉁함" 판단
            #   compactness = 4πA / P²
            #   - 1에 가까울수록 매끈한 원/사각형
            #   - 작을수록 울퉁불퉁/지글지글
            compactness = 4.0 * np.pi * contour_area / (peri ** 2 + 1e-6)

            # ▸ 다각형 근사로 직각/직선 형태인지 확인
            approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)
            num_vertices = len(approx)

            # 깔끔한 테두리/직사각형/선으로 볼 수 있는 조건
            is_smooth_boundary = compactness > 0.08       # 너무 엄격하지 않게
            is_rect_or_line = num_vertices <= 8           # 꼭짓점이 많으면 울퉁불퉁한 형상

            if not (is_smooth_boundary or is_rect_or_line):
                # 테두리가 너무 울퉁불퉁하면 노이즈로 판단
                continue

            # 위 조건을 통과한 경우에만 차선 후보로 유지
            clean[labels == i] = 255

        return clean

    # ---------------------------
    # 2) 트랙 ROI 적용
    # ---------------------------
    def apply_track_roi(self, binary):
        h, w = binary.shape[:2]
        mask = np.zeros_like(binary)

        pts = np.array([[
            (0, h),
            (0, int(h * 0.3)),
            (w, int(h * 0.3)),
            (w, h)
        ]], dtype=np.int32)

        cv2.fillPoly(mask, pts, 255)
        binary_roi = cv2.bitwise_and(binary, mask)

        return binary_roi, pts

    # ---------------------------
    # 3) 슬라이딩 윈도우로 좌/우 차선 찾기
    # ---------------------------
    def sliding_window(self, binary_warped):
        h, w = binary_warped.shape

        # 아래 30% 구간 히스토그램
        start_y = int(h * 0.7)
        histogram = np.sum(binary_warped[start_y:, :], axis=0)

        midpoint = histogram.shape[0] // 2
        left_hist = histogram[:midpoint]
        right_hist = histogram[midpoint:]

        leftx_base = np.argmax(left_hist)
        rightx_base = np.argmax(right_hist) + midpoint

        # 히스토그램 세기 기준으로 차선 유무 판단
        max_val = np.max(histogram) if np.max(histogram) > 0 else 1
        min_ratio = 0.25

        left_valid = np.max(left_hist) > max_val * min_ratio
        right_valid = np.max(right_hist) > max_val * min_ratio

        nonzero = binary_warped.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])

        window_height = h // self.n_windows

        left_lane_inds = []
        right_lane_inds = []

        leftx_current = leftx_base
        rightx_current = rightx_base

        for window in range(self.n_windows):
            win_y_low = h - (window + 1) * window_height
            win_y_high = h - window * window_height

            # 왼쪽
            if left_valid:
                win_xleft_low = leftx_current - self.margin
                win_xleft_high = leftx_current + self.margin

                good_left_inds = (
                    (nonzero_y >= win_y_low) &
                    (nonzero_y < win_y_high) &
                    (nonzero_x >= win_xleft_low) &
                    (nonzero_x < win_xleft_high)
                ).nonzero()[0]

                left_lane_inds.append(good_left_inds)

                if len(good_left_inds) > self.minpix:
                    leftx_current = int(np.mean(nonzero_x[good_left_inds]))

            # 오른쪽
            if right_valid:
                win_xright_low = rightx_current - self.margin
                win_xright_high = rightx_current + self.margin

                good_right_inds = (
                    (nonzero_y >= win_y_low) &
                    (nonzero_y < win_y_high) &
                    (nonzero_x >= win_xright_low) &
                    (nonzero_x < win_xright_high)
                ).nonzero()[0]

                right_lane_inds.append(good_right_inds)

                if len(good_right_inds) > self.minpix:
                    rightx_current = int(np.mean(nonzero_x[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds) if left_valid and len(left_lane_inds) > 0 else np.array([], dtype=int)
        right_lane_inds = np.concatenate(right_lane_inds) if right_valid and len(right_lane_inds) > 0 else np.array([], dtype=int)

        leftx = nonzero_x[left_lane_inds]
        lefty = nonzero_y[left_lane_inds]
        rightx = nonzero_x[right_lane_inds]
        righty = nonzero_y[right_lane_inds]

        left_fit, right_fit = None, None

        if left_valid and len(leftx) > 200:
            left_fit = np.polyfit(lefty, leftx, 2)
        if right_valid and len(rightx) > 200:
            right_fit = np.polyfit(righty, rightx, 2)

        return left_fit, right_fit

    # ---------------------------
    # 4) 메인 감지 + 시리얼 송신
    # ---------------------------
    def detect_lane(self, frame):
        """
        frame: BGR 이미지
        return: (output_img, offset_pixels, cmd)
        """
        h, w = frame.shape[:2]

        # 1) 이진화
        binary = self.color_binary(frame)

        # 2) ROI
        binary, roi_pts = self.apply_track_roi(binary)

        # 3) 차선 곡선
        left_fit, right_fit = self.sliding_window(binary)

        out_img = np.dstack((binary, binary, binary))

        # 곡선 시각화 범위
        y_min = int(h * 0.55)
        ploty = np.linspace(y_min, h - 1, h - y_min)

        lane_center_x = None

        # y_eval: 중앙/offset 계산 기준 y (near)
        y_eval = int(h * 0.9)
        left_x_eval = None
        right_x_eval = None

        # 🔥 커브 강도 계산용 변수 (near vs far 중앙 차이)
        curve_dx = None
        turn_strength = 0  # -2 ~ +2 정도로 스케일링

        # 왼쪽 차선
        if left_fit is not None:
            left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
            pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))], dtype=np.int32)
            cv2.polylines(out_img, pts_left, False, (0, 255, 0), 8)

            # y_eval에서의 왼쪽 x
            left_x_eval = left_fit[0] * y_eval**2 + left_fit[1] * y_eval + left_fit[2]

        # 오른쪽 차선
        if right_fit is not None:
            right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
            pts_right = np.array([np.transpose(np.vstack([right_fitx, ploty]))], dtype=np.int32)
            cv2.polylines(out_img, pts_right, False, (0, 0, 255), 8)

            # y_eval에서의 오른쪽 x
            right_x_eval = right_fit[0] * y_eval**2 + right_fit[1] * y_eval + right_fit[2]

        # 양쪽 선 감지 여부
        left_detected = left_x_eval is not None
        right_detected = right_x_eval is not None

        # 5) 차선 중앙 & 차선 폭 업데이트 (양쪽 다 있을 때만)
        if left_x_eval is not None and right_x_eval is not None:
            # 양쪽 다 보이는 경우 → 폭 측정 & 중앙 계산
            current_width = right_x_eval - left_x_eval
            if current_width > 0:
                if self.lane_width_px is None:
                    self.lane_width_px = current_width
                else:
                    # EMA 방식으로 천천히 업데이트
                    self.lane_width_px = 0.9 * self.lane_width_px + 0.1 * current_width

            lane_center_near = (left_x_eval + right_x_eval) / 2.0
            lane_center_x = int(lane_center_near)

            # 🔥 커브 강도 계산: 먼 지점의 중앙과 비교
            y_far = int(h * 0.6)
            left_far = left_fit[0] * y_far**2 + left_fit[1] * y_far + left_fit[2]
            right_far = right_fit[0] * y_far**2 + right_fit[1] * y_far + right_fit[2]
            lane_center_far = (left_far + right_far) / 2.0

            curve_dx = lane_center_far - lane_center_near  # >0: 먼 쪽이 오른쪽, <0: 먼 쪽이 왼쪽

            # 50px 당 1단계, -2 ~ +2 사이로 클램프
            turn_strength = int(max(-2, min(2, curve_dx / 50.0)))

        # ❗ 한쪽만 보이는 경우에는 더 이상 lane_center_x를 가상으로 만들지 않음
        #    → left_detected/right_detected 상태를 그대로 사용해서 바로 회전 판단

        # car 중심과 offset 계산 (양쪽 다 있는 경우에만)
        car_center_x = w // 2
        offset = lane_center_x - car_center_x if lane_center_x is not None else None

        # -----------------------
        # 🔥 시리얼 명령 결정 (+ 6초 버티기 로직 + 한쪽만일 때 L/R)
        # -----------------------
        cmd = "F"  # 기본: 중앙이라고 가정
        now = time.time()

        both_missing = (left_fit is None and right_fit is None)

        if both_missing:
            # 선이 완전히 안 보이는 상태
            if self.last_cmd in ("L", "R"):
                # 바로 이전에 L 또는 R을 주고 있던 중이라면 → 6초까지는 그 방향 유지
                if self.lost_start_time is None:
                    self.lost_start_time = now  # 처음 사라진 시점 기록

                elapsed = now - self.lost_start_time
                if elapsed < 6.0:
                    cmd = self.last_cmd   # 6초까지는 이전 조향 유지
                else:
                    cmd = "B"             # 6초 동안 안 잡히면 그때 후진
            else:
                # 이전 명령이 L/R이 아니면 기존 로직처럼 바로 후진
                cmd = "B"
        else:
            # 선이 하나라도 보이면 → lost 타이머 리셋
            self.lost_start_time = None

            # 🔥 한쪽 차선만 보이는 경우: 화면 기준으로 방향 결정
            if left_detected and not right_detected:
                # 중앙선 기준 왼쪽 차선만 보임 → 우회전
                cmd = "R"
            elif right_detected and not left_detected:
                # 중앙선 기준 오른쪽 차선만 보임 → 좌회전
                cmd = "L"
            else:
                # 양쪽 다 보이는 경우 → offset 기반 L/F/R 결정 (기존 로직 유지)
                if offset is not None:
                    # 노란 점이 왼쪽(화면 기준) → 차가 오른쪽으로 치우침 → R
                    if offset < -20:
                        cmd = "R"
                    # 노란 점이 오른쪽 → 차가 왼쪽으로 치우침 → L
                    elif offset > 20:
                        cmd = "L"
                    else:
                        cmd = "F"
                else:
                    # offset 계산 안 될 때는 일단 F 유지
                    cmd = "F"

        # 시리얼 전송 (내부 자동 전송 사용하는 경우에만)
        if self.ser is not None and self.use_internal_serial:
            try:
                self.ser.write(cmd.encode())  # 한 글자만 전송
            except Exception as e:
                print("[WARN] Serial write 실패:", e)

        # 디버깅 출력
        print(
            f"cmd={cmd}, offset={offset}, lane_width_px={self.lane_width_px}, "
            f"both_missing={both_missing}, last_cmd={self.last_cmd}, "
            f"curve_dx={curve_dx}, turn_strength={turn_strength}"
        )

        # 이번 프레임에서 보낸 명령을 last_cmd로 저장
        self.last_cmd = cmd

        # 시각화
        if lane_center_x is not None:
            cv2.circle(out_img, (int(lane_center_x), y_eval), 12, (0, 255, 255), -1)

        cv2.line(out_img, (car_center_x, h), (car_center_x, y_min), (255, 255, 255), 4)

        # 텍스트로 offset / cmd / curve_dx / turn_strength 표시
        if offset is not None:
            text = f"offset: {offset:+d}px  cmd: {cmd}"
        else:
            text = f"offset: N/A  cmd: {cmd}"
        cv2.putText(out_img, text, (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (255, 255, 255), 2)

        # 두 번째 줄: 커브 정보
        text2 = f"curve_dx: {curve_dx if curve_dx is not None else 'N/A'}  turn_strength: {turn_strength}"
        cv2.putText(out_img, text2, (30, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (255, 255, 255), 2)

        result = cv2.addWeighted(frame, 0.7, out_img, 0.9, 0)
        cv2.polylines(result, roi_pts, True, (0, 255, 0), 2)

        # 디버깅용 바이너리 표시 (모니터 있을 때만)
        if USE_GUI:
            dbg = cv2.resize(binary, None, fx=1.0, fy=1.0)
            cv2.imshow("binary", dbg)

        return result, offset, cmd


# ---------------------------
# 메인: 카메라 실시간 테스트
# ---------------------------
if __name__ == "__main__":
    # 시리얼 포트: /dev/ttyACM0, /dev/ttyACM1 등 상황에 맞게 수정
    detector = LaneDetectorSW(serial_port="/dev/ttyACM0", baud=9600)

    # 🔥 카메라 열기 (필요하면 0 → 1 등으로 변경)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다. 인덱스/연결 상태 확인 필요.")
        exit(1)

    print("[INFO] q 키를 누르면 종료합니다.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] 프레임을 읽을 수 없습니다.")
            break

        out, offset, cmd = detector.detect_lane(frame)

        if USE_GUI:
            show = cv2.resize(out, None, fx=1.0, fy=1.0)
            cv2.imshow("Lane Detection (Camera)", show)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:   # q 또는 ESC
                break
        else:
            # 모니터 없으면 키 입력도 없으니까 그냥 로직만 계속 돈다 (서비스용)
            pass

    cap.release()
    if USE_GUI:
        cv2.destroyAllWindows()
