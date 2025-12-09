# 📘 **README.md — Jetson + Arduino 기반 자율주행 미니카 프로젝트**

> **YOLO 신호등 인식 + 차선 인식 + 초음파 장애물 감지 + 서보 제어 + Jetson–Arduino 양방향 통신 시스템**
>
> 이 프로젝트는 **ChatGPT 및 다양한 AI 도구**와 협력하여 설계·구현되었습니다.

---

# 🚗 **프로젝트 개요**

이 시스템은 **Jetson Orin Nano**에서 카메라 기반 인식을 수행하고, **Arduino UNO**에서 모터와 초음파 센서를 제어하는 **하이브리드 자율주행 구조**로 설계되었습니다.

주요 기능:

| 기능                           | 플랫폼                      | 설명                               |
| ---------------------------- | ------------------------ | -------------------------------- |
| 신호등 색 인식(RED / GREEN / BLUE) | Jetson (Python + YOLO)   | 연속 감지 기반 상태 머신                   |
| 차선 감지 및 방향 명령 생성             | Jetson (Python + OpenCV) | 이진화·슬라이딩 윈도우·노이즈 필터링             |
| 장애물 감지(초음파)                  | Arduino UNO              | 20cm 이하 감지 시 즉시 정지               |
| 모터 제어(서보 2개)                 | Arduino UNO              | 전진·좌회전·우회전·후진                    |
| Jetson ↔ Arduino 통신          | Serial                   | 아두이노가 초음파로 멈추면 Jetson에도 즉시 알림(S) |

---

# 🧱 **전체 시스템 블록 다이어그램**

아래는 이 프로젝트의 전체 신호 흐름을 단순하게 표현한 구조도입니다.

```
┌────────────────────────┐
│     USB Camera #1      │
│ (Traffic Light Detect) │
└───────────┬────────────┘
            │ Frame
            ▼
     ┌──────────────┐
     │   YOLOv8n    │
     │  (best.pt)   │
     └───────┬──────┘
             │ Detected Color
             ▼
┌──────────────────────────────┐
│   Traffic Light StateMachine │
│ (NORMAL / RED_STOP / GO etc) │
└───────────┬──────────────────┘
            │ Command(F/L/R/S)
            ▼
┌──────────────────────────────┐
│       Serial Send (Jetson)   │
└───────────┬──────────────────┘
            │
            ▼
     ┌──────────────┐
     │   Arduino     │
     │  (Servo Ctrl) │
     └──────────────┘
            ▲
            │ S(Stop) feedback
┌───────────┴──────────────────┐
│  Ultrasonic Sensor (HC-SR04) │
│  - 20cm 이하 → 즉시 S 전송   │
│  - 20cm 초과 3회 → 재개      │
└──────────────────────────────┘
```

---

# 🛠 사용 기술

## ✔ 개발 도구

* **Arduino IDE**

  * 서보 제어 코드 작성
  * 초음파 센서 로직 구현
  * Jetson ↔ Arduino 시리얼 프로토콜 처리

* **Visual Studio Code (Jetson)**

  * YOLO 신호등 인식
  * OpenCV 차선 인식
  * Python 상태 머신 구성
  * 서비스 자동 실행(systemd) 구성

---

# 🤖 AI 협업 개발

이 프로젝트는
**ChatGPT(OpenAI)**와 지속적인 대화를 통해

* 알고리즘 설계
* 하드웨어 동작 방식 최적화
* 코드 리팩토링
* 디버깅
* systemd 서비스 구성
  을 전부 함께 진행하며 만들어졌습니다.

---

# 🔍 Jetson Python 로직 개요

## 1️⃣ YOLO 신호등 인식 알고리즘

* RED/BLUE/GREEN 분류
* RED는 **연속 2회** 감지해야 정지
* BLUE는 최근 차선 방향 좌우 통계를 기반으로 **지속 시간 적응형(1초 또는 2초)**

## 2️⃣ 차선 감지 알고리즘

* HSV + Adaptive Threshold 기반 이진화
* 모폴로지, 연결 요소 분석으로 노이즈 제거
* 양쪽 차선이 보이면 오프셋 기반 조향
* 한쪽만 보이면 그 방향으로 조향
* 최근 6개 조향 명령을 저장해 적응형 방향 보정

## 3️⃣ 아두이노로 명령 전송

Jetson이 초당 수십번 명령을 반복 전송
→ Arduino가 항상 최신 명령을 유지
→ 명령 누락/지연 없는 실시간 시스템

## 4️⃣ Arduino에서 보내는 "S" 피드백 처리

초음파로 멈춘 경우 Arduino가 Jetson에게
→ **"S"를 보내서 Jetson도 정지 상태로 전환**
(시리얼 버퍼 overflow 문제 해결)

---

# 🔧 Arduino 로직 개요

## 1️⃣ 초음파 안전 정지 알고리즘

* 3cm 이하 값은 노이즈 → 무시
* 20cm 이하 값이 **2회 이상 연속** 감지되면 즉시 정지(S)
* 정지 상태에서는 Jetson의 명령을 **무시**
* 20cm 초과 값이 **3회 연속** 나오면 다시 주행 재개
* 정지 시 Jetson에게도 `"S"` 전송 (동기화)

## 2️⃣ 서보 제어 로직

* 전진 / 후진 / 좌회전 / 우회전 펄스 값 정밀 설정
* 회전 시작 후 0.6초 동안 **안쪽 바퀴 고정(1500µs)**
  → 무거운 차체에서 회전이 잘 안 먹는 문제 해결

## 3️⃣ 부드러운 감속 알고리즘

정지(S) 명령이 오면:

* 800ms 동안 10단계로 점진적 감속
* 안정적이고 부드러운 정지 구현

---

# 🔧 **프로젝트 설치 및 실행 방법**

---

# 📦 1. Jetson Orin Nano — Python 환경 구성

## (1) YOLO 환경 설치

```bash
sudo apt update
sudo apt install python3-pip
pip install ultralytics opencv-python pyserial
```

## (2) 프로젝트 디렉토리 구성

```
/home/main/yoloenv/
 ├── test.py
 ├── line_rate_detect.py
 ├── best.pt
 ├── run_test.sh
```

`run_test.sh`:

```bash
#!/bin/bash
source /home/main/yoloenv/bin/activate
python3 /home/main/yoloenv/test.py
```

권한 부여:

```bash
chmod +x run_test.sh
```

---

# 🚀 2. Jetson에서 실행하기

```bash
python3 test.py
```

---

# 🔁 3. Jetson에서 서비스로 자동 실행하기(systemd)

`/etc/systemd/system/jetson-car.service`:

```ini
[Unit]
Description=Jetson Self-Driving Car Orchestrator (YOLO + Lane + Arduino)
After=network.target

[Service]
Type=simple
ExecStart=/home/main/yoloenv/run_test.sh
WorkingDirectory=/home/main/yoloenv
User=main
Restart=always

[Install]
WantedBy=multi-user.target
```

활성화:

```bash
sudo systemctl daemon-reload
sudo systemctl enable jetson-car.service
sudo systemctl start jetson-car.service
```

상태 확인:

```bash
sudo systemctl status jetson-car.service
```

---

# 🔌 4. Arduino 설치 & 업로드

## (1) Arduino IDE 설치

(공식 홈페이지 설치)

## (2) 보드 선택

```
Tools → Board → Arduino UNO
Tools → Port → /dev/ttyACM0
```

## (3) 현재 사용하는 전체 Arduino 코드 그대로 업로드

너에게 제공했던 **초음파 + 서보 + LED + 감속 + 통신 포함 전체 코드** 사용

---

# 📷 카메라 장착 & 배선

* USB Webcam 1개: 신호등 탐지용
* USB Webcam 1개: 차선 탐지용
* Arduino ↔ Jetson: USB 케이블 연결
* 초음파 센서:

  * Trig → D3
  * Echo → D4
  * GND → GND 공유
  * VCC → 5V

---
