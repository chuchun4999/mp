# file: line_follower_node.py

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import cv2
import numpy as np
import serial
import time

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        # ROS 2 파라미터를 사용하여 기존 argparse의 기능을 대체
        self.declare_parameter('video_device', 2)
        self.declare_parameter('threshold', 150)
        self.declare_parameter('use_hsv', False)
        self.declare_parameter('roi_top_ratio', 0.6)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('deadband', 30)

        # 선언된 파라미터의 초기값을 가져오기
        self.video_device = self.get_parameter('video_device').value
        self.threshold = self.get_parameter('threshold').value
        self.use_hsv = self.get_parameter('use_hsv').value
        self.roi_top_ratio = self.get_parameter('roi_top_ratio').value
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.deadband = self.get_parameter('deadband').value

        # 파라미터가 동적으로 변경될 때 호출될 콜백 함수를 등록
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.arduino = self.connect_arduino()
        self.cap = cv2.VideoCapture(self.video_device)
        if not self.cap.isOpened():
            self.get_logger().error(f"카메라 열기 실패 (/dev/video{self.video_device})")
            rclpy.shutdown()
            return
        self.get_logger().info(f"카메라 연결 성공! (/dev/video{self.video_device}) | 종료: q")

        # 30fps (1/30초 마다)로 timer_callback 함수를 실행합니다.
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'threshold':
                self.threshold = param.value
            elif param.name == 'use_hsv':
                self.use_hsv = param.value
            elif param.name == 'roi_top_ratio':
                self.roi_top_ratio = param.value
        self.get_logger().info(f"파라미터 변경됨: th={self.threshold}, use_hsv={self.use_hsv}")
        return SetParametersResult(successful=True)

    def connect_arduino(self):
        try:
            arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"아두이노 연결 성공: {self.serial_port} @ {self.baud_rate}")
            return arduino
        except Exception as e:
            self.get_logger().warn(f"아두이노 연결 실패: {e}")
            return None

    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("프레임 읽기 실패")
            return

        h, w, _ = frame.shape

        # --- 이진화 ---
        if self.use_hsv:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            binary = cv2.inRange(hsv, (0, 0, 200), (179, 60, 255))
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY)

        # --- ROI ---
        y0 = int(h * self.roi_top_ratio)
        roi = binary[y0:, :]

        # --- 중심 계산 ---
        pts = cv2.findNonZero(roi)
        direction = 'X'
        cx_visual = None
        white_count = 0 if pts is None else len(pts)

        if pts is not None and white_count > 0:
            cx = int(np.mean(pts[:, 0, 0]))
            cx_visual = cx
            dev = cx - (w // 2)

            if dev < -self.deadband:
                direction = 'L'
            elif dev > self.deadband:
                direction = 'R'
            else:
                direction = 'S'
        else:
            direction = 'X'

        # --- 아두이노 전송 ---
        if self.arduino:
            try:
                self.arduino.write(direction.encode())
            except Exception as e:
                self.get_logger().error(f"아두이노 전송 오류: {e}")
                self.arduino = None # 오류 발생 시 연결 끊기

        # --- 시각화 ---
        vis = frame.copy()
        cv2.rectangle(vis, (0, y0), (w, h), (255, 0, 0), 2)
        cv2.line(vis, (w//2, 0), (w//2, h), (0, 255, 0), 1)
        if cx_visual is not None:
            cv2.circle(vis, (cx_visual, (y0 + h)//2), 6, (0, 0, 255), -1)
        cv2.putText(vis, f"dir:{direction} whites:{white_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        
        cv2.imshow("frame", vis)
        cv2.imshow("binary", binary)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("'q' key pressed. Shutting down.")
            self.destroy_node()
            rclpy.shutdown()

    def on_shutdown(self):
        """노드가 종료될 때 호출되는 함수"""
        self.get_logger().info("노드 종료 중...")
        self.cap.release()
        cv2.destroyAllWindows()
        if self.arduino:
            self.arduino.close()

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
