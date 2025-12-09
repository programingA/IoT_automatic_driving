# test.py  (dual_camera 통합 코드)

import cv2
import time
from collections import deque
from ultralytics import YOLO
from line import LaneDetectorSW
import os  # 🔥 DISPLAY 확인용

# 🔥 DISPLAY 환경변수가 있을 때만 GUI 사용
USE_GUI = bool(os.environ.get("DISPLAY"))


def init_camera(index, width=1280, height=720):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f"[ERROR] Camera {index} 열기 실패")
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def draw_fps(frame, fps):
    cv2.putText(frame, f"FPS: {fps:.1f}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (0, 255, 0), 2, cv2.LINE_AA)


# 🔥 아두이노에서 보내는 ULTRA_BLOCK_ON / OFF 읽어서 플래그 갱신
def poll_arduino_status(ser, arduino_blocked):
    """
    ser: lane_detector.ser (Arduino 시리얼)
    arduino_blocked: 이전 상태
    return: 갱신된 arduino_blocked
    """
    if ser is None:
        return arduino_blocked

    try:
        # 버퍼에 쌓인 로그/상태 모두 처리
        while ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            # 디버깅용 전체 출력
            # print(f"[ARDUINO] {line}")
            if line == "[ULTRA_BLOCK_ON]":
                arduino_blocked = True
                print("[JETSON] Arduino ULTRA_BLOCK_ON → 명령 전송 정지")
            elif line == "[ULTRA_BLOCK_OFF]":
                arduino_blocked = False
                print("[JETSON] Arduino ULTRA_BLOCK_OFF → 명령 전송 재개")
    except Exception as e:
        print("[JETSON] serial read error:", e)

    return arduino_blocked


def main():
    # 1) YOLO 모델 로드 ----------------------------------------
    model_path = "best.pt"
    print(f"[INFO] YOLO model loading: {model_path}")
    model = YOLO(model_path)

    # 2) 차선 감지 객체 ---------------------------------------
    # 🔥 내부 시리얼 전송은 끄고, 명령만 계산하게 사용
    lane_detector = LaneDetectorSW(use_internal_serial=False)

    # 3) 카메라 두 개 열기 ------------------------------------
    cam_ai_index = 2      # 신호등
    cam_lane_index = 0    # 차선

    cap_ai = init_camera(cam_ai_index)
    cap_lane = init_camera(cam_lane_index)

    if cap_ai is None or cap_lane is None:
        print("[ERROR] 카메라 초기화 실패")
        return

    prev_time = time.time()
    fps = 0.0

    print("[INFO] q 키를 누르면 종료합니다.")

    # 🔥 신호등 상태 머신
    state = "NORMAL"      # NORMAL, RED_STOP, GREEN_GO, BLUE_RIGHT
    state_timer = None

    # 🔥 연속 프레임 카운터
    green_count = 0
    blue_count = 0
    required_frames = 5   # green/blue 연속 프레임 수

    # 🔥 최근 차선 명령 20개 기록 (L/R 비율 판단용)
    lane_cmd_history = deque(maxlen=6)

    # 🔥 이번 빨간불에서 보정(반대값 3번)을 이미 했는지 여부
    red_correction_done = False

    # 🔥 BLUE_RIGHT 유지 시간 (기본 2초, 조건에 따라 1초로 단축)
    blue_right_duration = 2.0

    # 🔥 아두이노가 초음파로 자체 정지 중인지 여부
    arduino_blocked = False

    while True:
        ret_ai, frame_ai = cap_ai.read()
        ret_lane, frame_lane = cap_lane.read()

        if not ret_ai and not ret_lane:
            print("[ERROR] 두 카메라 프레임 없음")
            break

        # 🔥 매 프레임마다 아두이노 상태 폴링
        arduino_blocked = poll_arduino_status(lane_detector.ser, arduino_blocked)

        detected_color = None  # "red", "green", "blue" 중 하나

        # =====================================================
        # 5) YOLO 신호등 탐지
        # =====================================================
        if ret_ai:
            results = model(frame_ai, verbose=False, conf=0.7)[0]
            annotated = results.plot()

            now = time.time()
            dt = now - prev_time
            if dt > 0:
                fps = 1.0 / dt
            prev_time = now

            if USE_GUI:
                draw_fps(annotated, fps)
                cv2.imshow("AI Camera - Traffic Light", annotated)

            for box in results.boxes:
                cls_id = int(box.cls[0])
                cls_name = results.names[cls_id]

                if cls_name == "red":
                    detected_color = "red"
                elif cls_name == "green":
                    detected_color = "green"
                elif cls_name == "blue":
                    detected_color = "blue"

        # =====================================================
        # 6) 신호등 상태 머신 (1초 유지 버전 + 빨간불 보정 1회)
        # =====================================================
        now = time.time()

        # NORMAL 상태
        if state == "NORMAL":
            if detected_color == "red":
                # 🔥 이번 빨간불에서 아직 보정 안 했을 때만 반대 방향 3번
                if lane_detector.ser is not None and not red_correction_done and not arduino_blocked:
                    L_count = sum(1 for c in lane_cmd_history if c == "L")
                    R_count = sum(1 for c in lane_cmd_history if c == "R")

                    print(f"[RED] history L={L_count}, R={R_count}")

                    opp_cmd = None
                    if L_count > R_count:
                        # L이 더 많았으면 → 차가 왼쪽으로 많이 꺾여 있었던 것 → R 3번 보정
                        opp_cmd = b"R"
                    elif R_count > L_count:
                        # R이 더 많았으면 → 차가 오른쪽으로 많이 꺾여 있었던 것 → L 3번 보정
                        opp_cmd = b"L"

                    if opp_cmd is not None:
                        lane_detector.ser.write(opp_cmd)
                        print(f"[RED CORRECTION] send {opp_cmd.decode()} (1/3)")
                    else:
                        print("[RED CORRECTION] L/R 비슷해서 보정 스킵 (직접 정지만 수행)")

                    # 이 빨간불에 대해서는 보정 완료 플래그
                    red_correction_done = True

                    # 마지막에는 무조건 정지
                    lane_detector.ser.write(b"S")
                    print("[RED] correction done → S(정지) 전송")
                else:
                    # 이미 보정한 빨간불이거나, 아두이노가 자체 정지 상태면
                    # 그냥 바로 정지 명령만 (단, 아두이노가 막혀 있으면 굳이 안 보내도 됨)
                    if lane_detector.ser is not None and not arduino_blocked:
                        lane_detector.ser.write(b"S")
                        print("[RED] 이미 보정 완료 → S(정지)만 전송")

                print("[STATE] NORMAL → RED_STOP (최근 20개 L/R 기반 보정 후 정지 또는 단순 정지)")
                state = "RED_STOP"
                state_timer = None
                green_count = 0
                blue_count = 0

            elif detected_color == "green":
                # 처음부터 green 감지 → 1초간 직진
                if lane_detector.ser is not None and not arduino_blocked:
                    lane_detector.ser.write(b"F")
                print("[STATE] NORMAL → GREEN_GO (1초 전진)")
                state = "GREEN_GO"
                state_timer = now
                green_count = 0
                blue_count = 0

            elif detected_color == "blue":
                # 처음부터 blue 감지 → 1초간 우회전
                if lane_detector.ser is not None and not arduino_blocked:
                    lane_detector.ser.write(b"R")
                print("[STATE] NORMAL → BLUE_RIGHT (1초 우회전)")
                state = "BLUE_RIGHT"
                state_timer = now
                green_count = 0
                blue_count = 0

        # RED_STOP 상태
        elif state == "RED_STOP":
            # red면 계속 정지 명령 (보정 없이 그냥 S만 유지)
            if detected_color == "red":
                if lane_detector.ser is not None and not arduino_blocked:
                    lane_detector.ser.write(b"S")

            # green / blue 연속 카운트
            if detected_color == "green":
                green_count += 1
                blue_count = 0
            elif detected_color == "blue":
                blue_count += 1
                green_count = 0
            else:
                green_count = 0
                blue_count = 0

            # 연속 프레임 도달 시 1초 동작 상태로 전환
            if green_count >= required_frames:
                print("[STATE] RED_STOP → GREEN_GO (1초 전진)")
                if lane_detector.ser is not None and not arduino_blocked:
                    lane_detector.ser.write(b"F")
                state = "GREEN_GO"
                state_timer = now
                # 🔥 다음 신호등을 위해 보정 플래그 리셋
                red_correction_done = False

            elif blue_count >= required_frames:
                print("[STATE] RED_STOP → BLUE_RIGHT (우회전)")
                if lane_detector.ser is not None and not arduino_blocked:
                    lane_detector.ser.write(b"R")

                # 🔥 여기서 최근 6개 중 R이 더 많으면 BLUE_RIGHT를 1초, 아니면 2초로
                L_count = sum(1 for c in lane_cmd_history if c == "L")
                R_count = sum(1 for c in lane_cmd_history if c == "R")
                if R_count > L_count:
                    blue_right_duration = 1.0
                    print(f"[BLUE DURATION] R>L → BLUE_RIGHT 1초")
                else:
                    blue_right_duration = 2.0
                    print(f"[BLUE DURATION] R<=L → BLUE_RIGHT 2초")

                state = "BLUE_RIGHT"
                state_timer = now
                # 🔥 다음 신호등을 위해 보정 플래그 리셋
                red_correction_done = False

        # GREEN_GO : 1초 동안 F 유지
        elif state == "GREEN_GO":
            if lane_detector.ser is not None and not arduino_blocked:
                lane_detector.ser.write(b"F")
            if now - state_timer >= 1.0:   # 1초
                print("[STATE] GREEN_GO → NORMAL")
                state = "NORMAL"

        # BLUE_RIGHT : blue_right_duration 동안 R 유지
        elif state == "BLUE_RIGHT":
            if lane_detector.ser is not None and not arduino_blocked:
                lane_detector.ser.write(b"R")
            if now - state_timer >= blue_right_duration:
                print("[STATE] BLUE_RIGHT → NORMAL")
                state = "NORMAL"

        # =====================================================
        # 7) 차선 감지 (신호등이 NORMAL일 때만 집행)
        # =====================================================
        if ret_lane:
            lane_vis, offset, lane_cmd = lane_detector.detect_lane(frame_lane)

            if USE_GUI:
                cv2.putText(lane_vis, f"offset: {offset}",
                            (30, 70), cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (0, 255, 255), 2)
                cv2.imshow("Lane Camera - Lane Detection", lane_vis)

            if state == "NORMAL":
                # NORMAL일 때만 히스토리 기록 + 명령 송신
                if lane_cmd in ("L", "R"):
                    lane_cmd_history.append(lane_cmd)

                if lane_detector.ser is not None and not arduino_blocked:
                    lane_detector.ser.write(lane_cmd.encode())
                print(f"[LANE CMD] → {lane_cmd}")

        # =====================================================
        # 8) 종료 키 처리
        # =====================================================
        if USE_GUI:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                if lane_detector.ser is not None:
                    lane_detector.ser.write(b"S")
                break
        else:
            # GUI 없으면 키 입력이 없으니 그냥 계속 루프 (서비스는 systemctl stop으로 종료)
            pass

    cap_ai.release()
    cap_lane.release()
    if USE_GUI:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
