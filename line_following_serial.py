import cv2
import numpy as np
import serial
import time

# 아두이노 포트에 맞게 조정 (ex: /dev/ttyACM0)
arduino = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)

cap = cv2.VideoCapture(0)  # 웹캠 번호에 따라 0 또는 1

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

    height, width = binary.shape
    roi = binary[int(height * 0.6):, :]  # 아래쪽 일부만 사용
    white_pixels = cv2.findNonZero(roi)

    direction = 'X'  # 기본 정지
    if white_pixels is not None:
        cx = int(np.mean(white_pixels[:, 0, 0]))
        deviation = cx - width // 2

        if deviation < -30:
            direction = 'L'
        elif deviation > 30:
            direction = 'R'
        else:
            direction = 'S'

    # 아두이노로 전송
    arduino.write(direction.encode())
    print("Direction:", direction)

    # 디버그 영상 출력
    cv2.imshow("Binary", binary)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
arduino.close()
cv2.destroyAllWindows()
