#include <Servo.h>

// --- 핀 정의 ---
const int LEFT_SERVO_PIN = 13;   
const int RIGHT_SERVO_PIN = 12; 
const int HEADLIGHT_PIN = 5;       
const int BRAKE_LIGHT_PIN = 6;    
const int LEFT_SIGNAL_PIN = 10;
const int RIGHT_SIGNAL_PIN = 9;
const int CDS_ANALOG_PIN = A0;      
const int BUZZER_PIN = 2; 

// 🔥 초음파 센서 핀 (HC-SR04 기준 예시: Trig=3, Echo=4)
const int ULTRA_TRIG_PIN = 3;
const int ULTRA_ECHO_PIN = 4;

// --- 상수 및 설정 ---
const int CDS_THRESHOLD = 300;    
const int DECELERATION_DURATION = 800;  // 감속 총 시간 (1초)
const int DECELERATION_STEPS = 10;      // 감속 단계 수
const unsigned long BUZZER_INTERVAL_MS = 500; 

// 🔥 초음파 관련 상수
const float ULTRA_STOP_CM = 15.0;     // 20cm 이하이면 위험
const float ULTRA_MIN_CM  = 3.0;      // 3cm 이하는 노이즈로 무시
const float ULTRA_MAX_CM  = 400.0;    // 너무 큰 값(예: 4m 이상)도 무시
const int   ULTRA_REQUIRED_HITS  = 2; // 20cm 이하가 2번 이상 감지되면 정지
const int   ULTRA_CLEAR_REQUIRED = 3; // 20cm 초과가 3번 연속 나오면 다시 출발 허용

// --- 서보 객체 생성 ---
Servo servoLeft;
Servo servoRight;

// --- 모터 제어 펄스 값 설정 ---
const int SERVO_STOP = 1500; 

// 전진
const int FORWARD_PULSE_LEFT = 1700;
const int FORWARD_PULSE_RIGHT = 1300;

// 좌회전 (왼쪽 느리게, 오른쪽 빠르게)
const int TURN_LEFT_PULSE_LEFT = 1510;  
const int TURN_LEFT_PULSE_RIGHT = 1300; 

// 우회전 (왼쪽 빠르게, 오른쪽 느리게)
const int TURN_RIGHT_PULSE_LEFT = 1700;
const int TURN_RIGHT_PULSE_RIGHT = 1490;

// 후진
const int REVERSE_PULSE_LEFT = 1300;  
const int REVERSE_PULSE_RIGHT = 1700; 

// --- 변수 ---
char currentCommand = 'S';  // 현재 명령
char lastCommand = 'S';     // 이전 명령
bool isReversing = false;

// 부드러운 감속 관련 변수
bool isDecelerating = false;
unsigned long decelStartTime = 0;
int decelStartPulseLeft = SERVO_STOP;
int decelStartPulseRight = SERVO_STOP;

// 후진 부저 관련 변수
unsigned long lastReverseBuzzTime = 0;

// 🔥 턴 시작 시점 기록 (L/R 명령용)
unsigned long turnStartTime = 0;

// 🔥 초음파 관련 변수
bool ultrasonic_block = false;      // 초음파로 인해 현재 정지 중인지
int  ultrasonicCloseCount = 0;      // 20cm 이하 연속 감지 횟수
int  ultrasonicFarCount   = 0;      // 20cm 초과 연속 감지 횟수

// --- 함수 선언 ---
void checkHeadlights();
void executeCommand(char cmd);
void updateLEDs(char cmd);
void buzzOn();  
void buzzOff(); 
void handleReverseBuzzer(); 
void initializeSensors();
void smoothDecelerate();
long readUltrasonicCm();  // 🔥 초음파 측정 함수 선언


void setup() {
  pinMode(HEADLIGHT_PIN, OUTPUT);
  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  pinMode(LEFT_SIGNAL_PIN, OUTPUT);
  pinMode(RIGHT_SIGNAL_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // 🔥 초음파 핀 설정
  pinMode(ULTRA_TRIG_PIN, OUTPUT);
  pinMode(ULTRA_ECHO_PIN, INPUT);

  servoLeft.attach(LEFT_SERVO_PIN);
  servoRight.attach(RIGHT_SERVO_PIN);
  
  Serial.begin(9600);   // 🔥 Jetson Python이랑 반드시 9600으로 맞추기!
  
  initializeSensors();
  
  // 초기 정지 상태
  servoLeft.writeMicroseconds(SERVO_STOP);
  servoRight.writeMicroseconds(SERVO_STOP);
  digitalWrite(BRAKE_LIGHT_PIN, HIGH);
  
  Serial.println("Arduino Ready - Jetson Control Mode");
  Serial.println("Commands: F=Forward, B=Back, L=Left, R=Right, J=AdjustLeft, K=AdjustRight, S=Stop");
}

void loop() {
  // 자동 헤드라이트
  checkHeadlights();
  
  // 후진 부저
  handleReverseBuzzer();
  
  // 감속 중이면 감속만 처리
  if (isDecelerating) {
    smoothDecelerate();
    return;
  }

  // 🔥 초음파 안전 정지 로직 (제슨 명령을 읽기 전에 먼저 체크)
  long ultraDist = readUltrasonicCm();
  bool ultraValid = (ultraDist >= ULTRA_MIN_CM && ultraDist <= ULTRA_MAX_CM);

  if (ultraValid && ultraDist <= ULTRA_STOP_CM) {
    // 3cm ~ 20cm 구간이 감지되면 위험 카운트 업
    ultrasonicCloseCount++;
    ultrasonicFarCount = 0;
  } else if (ultraValid && ultraDist > ULTRA_STOP_CM) {
    // 20cm 초과 유효값 → 안전 카운트 업
    ultrasonicFarCount++;
    ultrasonicCloseCount = 0;
  } else {
    // 유효하지 않은 값(-1, 노이즈 등)은 둘 다 리셋
    ultrasonicCloseCount = 0;
    ultrasonicFarCount   = 0;
  }

  // 20cm 이하가 2번 이상 연속 감지되면 초음파 블럭 활성화
  if (ultrasonicCloseCount >= ULTRA_REQUIRED_HITS) {
    ultrasonic_block = true;
    buzzOn();
  }

  // 이미 블럭 상태일 때, 20cm 초과 안전거리 유효값이
  // 연속 ULTRA_CLEAR_REQUIRED번 나오면 블럭 해제
  if (ultrasonic_block && ultraValid && ultraDist > ULTRA_STOP_CM) {
    if (ultrasonicFarCount >= ULTRA_CLEAR_REQUIRED) {
      ultrasonic_block = false;
      ultrasonicFarCount   = 0;
      ultrasonicCloseCount = 0;
      buzzOff();
      Serial.println("[ULTRA] Safe distance maintained, unblock and resume commands");
    }
  }

  if (ultrasonic_block) {
    // 초음파에 막힌 상태에서는 무조건 정지 명령만 실행하고
    // 제슨에서 오는 새 명령은 처리하지 않음
    currentCommand = 'S';
    updateLEDs('S');
    executeCommand('S');
    return;  // 이번 loop 종료, 다음 loop에서 다시 초음파 먼저 확인
  }
  
  // 시리얼 명령 수신
  if (Serial.available() > 0) {
    char newCommand = Serial.read();
    
    // Jetson에서 보내는 유효 명령: F, B, L, R (+ J, K, S 가능)
    if (newCommand == 'F' || newCommand == 'B' || newCommand == 'L' || 
        newCommand == 'R' || newCommand == 'J' || newCommand == 'K' || 
        newCommand == 'S') {
      
      if (newCommand != currentCommand) {
        Serial.print("Command changed: ");
        Serial.print(currentCommand);
        Serial.print(" -> ");
        Serial.println(newCommand);
        
        lastCommand = currentCommand;
        currentCommand = newCommand;
        
        // S(정지) 명령 + 직전이 움직이는 명령이면 감속 시작
        if (currentCommand == 'S' && 
            (lastCommand == 'F' || lastCommand == 'L' || lastCommand == 'R' || 
             lastCommand == 'J' || lastCommand == 'K')) {
          
          if (lastCommand == 'F') {
            decelStartPulseLeft = FORWARD_PULSE_LEFT;
            decelStartPulseRight = FORWARD_PULSE_RIGHT;
          } else if (lastCommand == 'L') {
            decelStartPulseLeft = TURN_LEFT_PULSE_LEFT;
            decelStartPulseRight = TURN_LEFT_PULSE_RIGHT;
          } else if (lastCommand == 'R') {
            decelStartPulseLeft = TURN_RIGHT_PULSE_LEFT;
            decelStartPulseRight = TURN_RIGHT_PULSE_RIGHT;
          }
          
          isDecelerating = true;
          decelStartTime = millis();
          Serial.println("Starting smooth deceleration...");
        }
        
        // 후진 상태 플래그
        isReversing = (currentCommand == 'B');
        
        // LED 업데이트
        updateLEDs(currentCommand);
      }
    } else {
      // 개행문자 등 무시
      Serial.print("Ignored invalid character: ");
      Serial.println((int)newCommand);
    }
  }
  
  // 현재 명령 실행
  executeCommand(currentCommand);
}


// ----------------------------------------------------------------------
// 초음파 거리 측정 (cm 단위, 실패 시 -1 반환)
// ----------------------------------------------------------------------
long readUltrasonicCm() {
  // Trig 펄스
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG_PIN, LOW);

  // Echo 펄스 길이 측정
  long duration = pulseIn(ULTRA_ECHO_PIN, HIGH, 30000UL); // 최대 30ms(약 5m)까지 대기
  if (duration == 0) {
    // 타임아웃 → 측정 실패
    return -1;
  }

  // 거리(cm) = (duration / 2) / 29.1
  long distance = (long)((duration / 2.0) / 29.1);
  return distance;
}


// ----------------------------------------------------------------------
// 부드러운 감속
// ----------------------------------------------------------------------
void smoothDecelerate() {
  unsigned long elapsedTime = millis() - decelStartTime;
  
  if (elapsedTime >= DECELERATION_DURATION) {
    isDecelerating = false;
    servoLeft.writeMicroseconds(SERVO_STOP);
    servoRight.writeMicroseconds(SERVO_STOP);
    Serial.println("Deceleration complete - Full stop");
    return;
  }
  
  float progress = (float)elapsedTime / (float)DECELERATION_DURATION;
  
  int currentPulseLeft = decelStartPulseLeft + (SERVO_STOP - decelStartPulseLeft) * progress;
  int currentPulseRight = decelStartPulseRight + (SERVO_STOP - decelStartPulseRight) * progress;
  
  servoLeft.writeMicroseconds(currentPulseLeft);
  servoRight.writeMicroseconds(currentPulseRight);
}


// ----------------------------------------------------------------------
// 모터 제어
// ----------------------------------------------------------------------
void executeCommand(char cmd) {
  switch (cmd) {
    case 'F':  // 전진
      servoLeft.writeMicroseconds(FORWARD_PULSE_LEFT);
      servoRight.writeMicroseconds(FORWARD_PULSE_RIGHT);
      break;
      
    case 'B':  // 후진
      servoLeft.writeMicroseconds(REVERSE_PULSE_LEFT);
      servoRight.writeMicroseconds(REVERSE_PULSE_RIGHT);
      break;
      
    case 'L':  // 좌회전
    {
      unsigned long elapsed = millis() - turnStartTime;
      // 🔥 턴 시작 후 0.4초 동안 안쪽 바퀴(왼쪽) 1500 고정
      if (elapsed < 600) {
        servoLeft.writeMicroseconds(1500);                    // 안쪽 바퀴 고정
        servoRight.writeMicroseconds(TURN_LEFT_PULSE_RIGHT);  // 바깥 바퀴 회전
      } else {
        // 이후에는 기존 값(1530 / 1300) 사용
        servoLeft.writeMicroseconds(TURN_LEFT_PULSE_LEFT);
        servoRight.writeMicroseconds(TURN_LEFT_PULSE_RIGHT);
      }
      break;
    }
      
    case 'R':  // 우회전
    {
      unsigned long elapsed = millis() - turnStartTime;
      // 🔥 턴 시작 후 0.4초 동안 안쪽 바퀴(오른쪽) 1500 고정
      if (elapsed < 600) {
        servoLeft.writeMicroseconds(TURN_RIGHT_PULSE_LEFT);   // 바깥 바퀴 회전
        servoRight.writeMicroseconds(1500);                   // 안쪽 바퀴 고정
      } else {
        // 이후에는 기존 값(1700 / 1470) 사용
        servoLeft.writeMicroseconds(TURN_RIGHT_PULSE_LEFT);
        servoRight.writeMicroseconds(TURN_RIGHT_PULSE_RIGHT);
      }
      break;
    } 
    
    case 'S':  // 정지
      if (!isDecelerating) {
        servoLeft.writeMicroseconds(SERVO_STOP);
        servoRight.writeMicroseconds(SERVO_STOP);
      }
      break;
      
    default:
      servoLeft.writeMicroseconds(SERVO_STOP);
      servoRight.writeMicroseconds(SERVO_STOP);
      break;
  }
}


// ----------------------------------------------------------------------
// LED 제어
// ----------------------------------------------------------------------
void updateLEDs(char cmd) {
  digitalWrite(BRAKE_LIGHT_PIN, LOW);
  digitalWrite(LEFT_SIGNAL_PIN, LOW);
  digitalWrite(RIGHT_SIGNAL_PIN, LOW);
  
  switch (cmd) {
    case 'F':
      break;
    case 'B':
      digitalWrite(BRAKE_LIGHT_PIN, HIGH);
      break;
    case 'L':
      digitalWrite(LEFT_SIGNAL_PIN, HIGH);
      break;
    case 'R':
      digitalWrite(RIGHT_SIGNAL_PIN, HIGH);
      break;
    case 'J':
      break;
    case 'K':
      break;
    case 'S':
      digitalWrite(BRAKE_LIGHT_PIN, HIGH);
      break;
  }
}


// ----------------------------------------------------------------------
// 초기화
// ----------------------------------------------------------------------
void initializeSensors() {
  digitalWrite(HEADLIGHT_PIN, LOW);
  digitalWrite(BRAKE_LIGHT_PIN, LOW);
  digitalWrite(LEFT_SIGNAL_PIN, LOW);
  digitalWrite(RIGHT_SIGNAL_PIN, LOW);
//  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.println("Sensors initialized");
}


// ----------------------------------------------------------------------
// 부저 제어
// ----------------------------------------------------------------------

void buzzOn() {
  digitalWrite(BUZZER_PIN, HIGH);
}

void buzzOff() {
  digitalWrite(BUZZER_PIN, LOW);
}

void handleReverseBuzzer() {
  if (isReversing) {
    unsigned long currentMillis = millis();
    
    if (currentMillis - lastReverseBuzzTime >= BUZZER_INTERVAL_MS) {
      lastReverseBuzzTime = currentMillis; 

      if (digitalRead(BUZZER_PIN) == HIGH) {
        buzzOff();
      } else {
        buzzOn();
      }
    }
  } else {
    buzzOff();
  }
}


// ----------------------------------------------------------------------
// 자동 헤드라이트
// ----------------------------------------------------------------------
void checkHeadlights() {
  int cdsValue = analogRead(CDS_ANALOG_PIN);
  
  if (cdsValue < CDS_THRESHOLD) {
    digitalWrite(HEADLIGHT_PIN, HIGH); 
  } else {
    digitalWrite(HEADLIGHT_PIN, LOW);
  }
}
