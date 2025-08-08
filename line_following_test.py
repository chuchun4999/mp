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

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=int, default=2, help="VideoCapture index (/dev/videoN)")
    parser.add_argument("--th", type=int, default=150, help="Gray threshold (0~255)")
    parser.add_argument("--use_hsv", action="store_true", help="Use HSV white mask instead of gray threshold")
    parser.add_argument("--roi_top", type=float, default=0.6, help="ROI top ratio (0~1)")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="Arduino serial port")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate")
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

        if pts is not None and white_count > 0:
            cx = int(np.mean(pts[:, 0, 0]))
            cx_visual = cx
            dev = cx - (w // 2)

            if dev < -30:
                direction = 'L'
            elif dev > 30:
                direction = 'R'
            else:
                direction = 'S'
        else:
            direction = 'X'

        # --- 5) 아두이노 전송 ---
        if arduino:
            try:
                arduino.write(direction.encode())
            except Exception as e:
                print(f"[ERROR] 아두이노 전송 오류: {e}")
                arduino = None  # 오류 발생 시 연결 끊기
        else:
            # 연결 안 된 상태에서 재시도 원하면 주석 해제
            # arduino = connect_arduino(args.port, args.baud)
            pass

        # --- 6) 원본 오버레이 ---
        vis = frame.copy()
        cv2.rectangle(vis, (0, y0), (w, h), (255, 0, 0), 2)
        cv2.line(vis, (w//2, 0), (w//2, h), (0, 255, 0), 1)
        if cx_visual is not None:
            cv2.circle(vis, (cx_visual, (y0 + h)//2), 6, (0, 0, 255), -1)
            cv2.line(vis, (cx_visual, y0), (cx_visual, h), (0, 0, 255), 1)
        cv2.putText(vis, f"dir:{direction}  whites:{white_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        cv2.putText(vis, f"mode:{'HSV' if args.use_hsv else 'GRAY'}  th:{args.th}  roi_top:{args.roi_top}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        # --- 7) binary 오버레이 ---
        bin_vis = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(bin_vis, (0, y0), (w, h), (255, 0, 0), 2)
        cv2.line(bin_vis, (w//2, 0), (w//2, h), (0, 255, 0), 1)
        if cx_visual is not None:
            cv2.circle(bin_vis, (cx_visual, (y0 + h)//2), 6, (0, 0, 255), -1)
            cv2.line(bin_vis, (cx_visual, y0), (cx_visual, h), (0, 0, 255), 1)
        cv2.putText(bin_vis, f"dir:{direction}  whites:{white_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        cv2.putText(bin_vis, f"mode:{'HSV' if args.use_hsv else 'GRAY'}  th:{args.th}  roi_top:{args.roi_top}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

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
