#include <TinyGPS++.h>

// 모터 드라이버 핀 설정
const int motorLeftInput1 = 2;
const int motorLeftInput2 = 3;
const int motorRightInput1 = 4;
const int motorRightInput2 = 5;
const int motorLeftEnable = 6;
const int motorRightEnable = 7;
const int shockPin = A0;  // 충격 감지 핀
const int shockThreshold = 512;  // 충격 감지 임계값

// GPS 모듈 설정
TinyGPSPlus gps;

void setup() {
    Serial.begin(9600);  // 라즈베리파이와 UART 통신
    Serial1.begin(9600); // GPS 모듈
    pinMode(motorLeftInput1, OUTPUT);
    pinMode(motorLeftInput2, OUTPUT);
    pinMode(motorRightInput1, OUTPUT);
    pinMode(motorRightInput2, OUTPUT);
    pinMode(motorLeftEnable, OUTPUT);
    pinMode(motorRightEnable, OUTPUT);
    pinMode(shockPin, INPUT);

    Serial.println("start serial");
}

void loop() {
    // 라즈베리파이에서 명령 수신
    if (Serial.available() > 0) {
        char command = Serial.read();  // 라즈베리파이에서 명령 수신

        switch (command) {
            case 'F': moveForward(); break;
            case 'L': turnLeft(); break;
            case 'R': turnRight(); break;
            case 'B': moveBackward(); break;
            case 'S': stopMotors(); break;
        }
    }

    // 충격 감지
    int shockValue = analogRead(shockPin);  // 충격 감지 값
    bool shockDetected = shockValue > shockThreshold;
    if (shockDetected) {
        // 충격이 감지되었을 때만 출력
        Serial.println(shockValue);  // 숫자 값만 출력 (시리얼 플로터에서 그래프 표시 가능)
    }

    // GPS 데이터 업데이트 및 시리얼 전송
    while (Serial1.available() > 0) {
        gps.encode(Serial1.read());
    }

    if (gps.location.isUpdated()) {
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();

        // GPS 좌표를 라즈베리파이로 전송
        Serial.print("GPS: ");
        Serial.print(latitude, 6);  // 위도
        Serial.print(", ");
        Serial.println(longitude, 6);  // 경도
    }
}

void moveForward() {
    analogWrite(motorLeftEnable, 255);
    analogWrite(motorRightEnable, 255);
    digitalWrite(motorLeftInput1, HIGH);
    digitalWrite(motorLeftInput2, LOW);
    digitalWrite(motorRightInput1, HIGH);
    digitalWrite(motorRightInput2, LOW);
}

void moveBackward() {
    analogWrite(motorLeftEnable, 255);
    analogWrite(motorRightEnable, 255);
    digitalWrite(motorLeftInput1, LOW);
    digitalWrite(motorLeftInput2, HIGH);
    digitalWrite(motorRightInput1, LOW);
    digitalWrite(motorRightInput2, HIGH);
}

void turnLeft() {
    analogWrite(motorLeftEnable, 150);
    analogWrite(motorRightEnable, 255);
    digitalWrite(motorLeftInput1, LOW);
    digitalWrite(motorLeftInput2, HIGH);
    digitalWrite(motorRightInput1, HIGH);
    digitalWrite(motorRightInput2, LOW);
}

void turnRight() {
    analogWrite(motorLeftEnable, 255);
    analogWrite(motorRightEnable, 150);
    digitalWrite(motorLeftInput1, HIGH);
    digitalWrite(motorLeftInput2, LOW);
    digitalWrite(motorRightInput1, LOW);
    digitalWrite(motorRightInput2, HIGH);
}

void stopMotors() {
    analogWrite(motorLeftEnable, 0);
    analogWrite(motorRightEnable, 0);
    digitalWrite(motorLeftInput1, LOW);
    digitalWrite(motorLeftInput2, LOW);
    digitalWrite(motorRightInput1, LOW);
    digitalWrite(motorRightInput2, LOW);
}
