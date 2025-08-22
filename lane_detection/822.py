import cv2
import numpy as np
import argparse
import sys
import serial
import time

def connect_arduino(port, baud):
    try:
        arduino = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # 연결 안정화 대기
        print(f"[OK] 아두이노 연결 성공: {port} @ {baud}")
        return arduino
    except Exception as e:
        print(f"[WARN] 아두이노 연결 실패: {e}")
        return None

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=int, default=2, help="VideoCapture index (/dev/videoN)")
    parser.add_argument("--th", type=int, default=150, help="Gray threshold (0~255)")
    parser.add_argument("--use_hsv", action="store_true", help="Use HSV white mask instead of gray threshold")
    parser.add_argument("--roi_top", type=float, default=0.6, help="ROI top ratio (0~1)")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="Arduino serial port")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate (아두이노와 동일)")
    # --- 추가: 속도/조향 파라미터 ---
    parser.add_argument("--base", type=int, default=120, help="기본 전진 PWM(0~255)")
    parser.add_argument("--maxpwm", type=int, default=200, help="최대 PWM 제한(0~255)")
    parser.add_argument("--kp", type=float, default=0.6, help="조향 P게인")
    parser.add_argument("--deadband", type=int, default=30, help="중앙 데드밴드(px)")
    args = parser.parse_args()

    # --- 0) 아두이노 연결 시도 ---
    arduino = connect_arduino(args.port, args.baud)

    # --- 1) 카메라 연결 ---
    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        print(f"[ERROR] 카메라 열기 실패(/dev/video{args.device})")
        sys.exit(1)
    print(f"[OK] 카메라 연결 성공! (/dev/video{args.device})  | 종료: q")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[WARN] 프레임 읽기 실패")
            continue

        h, w = frame.shape[:2]

        # --- 2) 이진화 ---
        if args.use_hsv:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # 밝은(흰색) 영역 마스크
            mask = cv2.inRange(hsv, (0, 0, 200), (179, 60, 255))
            binary = mask
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, args.th, 255, cv2.THRESH_BINARY)

        # --- 3) ROI ---
        y0 = int(h * args.roi_top)
        roi = binary[y0:, :].copy()

        # --- 4) 중심 계산 ---
        pts = cv2.findNonZero(roi)
        direction = 'X'
        cx_visual = None
        white_count = 0 if pts is None else len(pts)

        dev = 0
        if pts is not None and white_count > 0:
            cx = int(np.mean(pts[:, 0, 0]))
            cx_visual = cx
            dev = cx - (w // 2)  # +면 오른쪽
            if dev < -args.deadband:
                direction = 'L'
            elif dev > args.deadband:
                direction = 'R'
            else:
                direction = 'S'
        else:
            direction = 'X'  # 라인 못 찾음

        # --- 5) 좌/우 PWM 계산 및 전송 ---
        left_pwm = 0
        right_pwm = 0

        if direction == 'X':
            # 라인 못 찾을 때: 천천히 직진(혹은 정지/감속 로직)
            steer = 0
            left_pwm = right_pwm = int(args.base * 0.6)
        else:
            # 단순 P제어: 조향 = Kp * dev
            steer = args.kp * dev
            left_pwm  = int(clamp(args.base + steer, -args.maxpwm, args.maxpwm))
            right_pwm = int(clamp(args.base - steer, -args.maxpwm, args.maxpwm))

        # 아두이노로 송신: "L<left> R<right>\n"
        if arduino:
            try:
                msg = f"L{left_pwm} R{right_pwm}\n"
                arduino.write(msg.encode())
            except Exception as e:
                print(f"[ERROR] 아두이노 전송 오류: {e}")
                arduino = None

        # --- 6) 원본 오버레이 ---
        vis = frame.copy()
        cv2.rectangle(vis, (0, y0), (w, h), (255, 0, 0), 2)
        cv2.line(vis, (w//2, 0), (w//2, h), (0, 255, 0), 1)
        if cx_visual is not None:
            cv2.circle(vis, (cx_visual, (y0 + h)//2), 6, (0, 0, 255), -1)
            cv2.line(vis, (cx_visual, y0), (cx_visual, h), (0, 0, 255), 1)
        cv2.putText(vis, f"dir:{direction}  whites:{white_count}  dev:{dev}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        cv2.putText(vis, f"mode:{'HSV' if args.use_hsv else 'GRAY'}  th:{args.th}  roi_top:{args.roi_top}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(vis, f"BASE:{args.base} MAX:{args.maxpwm} Kp:{args.kp}",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        # --- 7) binary 오버레이 ---
        bin_vis = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(bin_vis, (0, y0), (w, h), (255, 0, 0), 2)
        cv2.line(bin_vis, (w//2, 0), (w//2, h), (0, 255, 0), 1)
        if cx_visual is not None:
            cv2.circle(bin_vis, (cx_visual, (y0 + h)//2), 6, (0, 0, 255), -1)
            cv2.line(bin_vis, (cx_visual, y0), (cx_visual, h), (0, 0, 255), 1)
        cv2.putText(bin_vis, f"dir:{direction}  whites:{white_count}  dev:{dev}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

        # --- 8) 출력 ---
        cv2.imshow("frame", vis)
        cv2.imshow("binary", bin_vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key in (ord('='), ord('+')):
            args.th = min(args.th + 5, 255)
        if key in (ord('-'), ord('_')):
            args.th = max(args.th - 5, 0)

    cap.release()
    cv2.destroyAllWindows()
    if arduino:
        arduino.close()

if __name__ == "__main__":
    main()
