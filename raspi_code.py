import cv2
import serial
import time
import torch
from ultralytics import YOLO

# 아두이노 시리얼 포트 설정
arduino = serial.Serial('/dev/ttyACM0', 9600) 
time.sleep(2)  # 시리얼 통신 안정화

# YOLO 모델 로드
model = YOLO("yolov5n.pt")

# 카메라 열기
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("❌ Failed to open camera")
    exit()

# 명령 전송 함수
def send_command(command):
    arduino.write(command.encode())  # 아두이노로 명령 전송
    print(f"Sent command: {command}")

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to grab frame")
        break

    frame_resized = cv2.resize(frame, (240,240))

    with torch.no_grad():
        results = model(frame_resized)

    # 기본값 (전진)
    command = 'F'
    obstacle_detected = False  # 장애물 감지 여부

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box 좌표
            confidence = box.conf[0].item()  # 신뢰도 점수
            class_id = int(box.cls[0].item())  # 클래스 ID
            label = model.names[class_id]  # 클래스 이름

            # 감지된 객체 표시
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label}: {confidence:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 장애물 감지 시 정지 후 후진
            if label in ["person", "car", "truck", "stop sign"]:
                obstacle_detected = True
                command = 'S'

    # 충격 감지 및 시리얼 데이터 읽기
    if arduino.in_waiting > 0:
        data = arduino.readline().decode('utf-8').strip()
        if data == "shock_detected":
            print("충격 감지됨")

        # GPS 값 처리
        if data.startswith("GPS:"):
            gps_data = data[5:].strip()  # "GPS: " 부분을 제거하고 좌표만 남김
            latitude, longitude = gps_data.split(", ")
            print(f"GPS 위치: 위도: {latitude}, 경도: {longitude}")

    # 장애물이 감지되면 일정 시간 후진
    if obstacle_detected:
        send_command('S')  #정지

    else:
        # 차선 감지 (좌측이면 좌회전, 우측이면 우회전)
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                label = model.names[class_id]

                if label == "lane_left":
                    command = 'L'
                elif label == "lane_right":
                    command = 'R'
                else:
                    command = 'F'  # 장애물이 없으면 전진

        send_command(command)  # 아두이노로 명령 전송

    # 결과 화면 출력
    cv2.imshow("YOLOv5 Object Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 종료 처리
cap.release()
cv2.destroyAllWindows()
arduino.close()
