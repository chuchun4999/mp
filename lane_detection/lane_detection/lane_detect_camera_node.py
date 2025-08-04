import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from .lane_detector import detect_lanes

class LaneDetectCamera(Node):
    def __init__(self):
        super().__init__('lane_detect_camera')
        self.bridge = CvBridge()
        self.raw_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.lane_pub = self.create_publisher(Image, 'camera/lane_image', 10)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('USB Camera open failed!')
            rclpy.shutdown()
            return
        self.get_logger().info('USB Camera opened.')
        self.timer = self.create_timer(1.0/30.0, self.loop_callback)


    def loop_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to grab frame')
            return

        # 원본 영상 발행
        raw_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        raw_img_msg.header.stamp = self.get_clock().now().to_msg()
        self.raw_pub.publish(raw_img_msg)

        # import한 함수를 호출하여 차선 검출 수행
        lane_image = detect_lanes(frame)

        # 결과 영상 발행
        lane_img_msg = self.bridge.cv2_to_imgmsg(lane_image, encoding='bgr8')
        lane_img_msg.header.stamp = self.get_clock().now().to_msg()
        self.lane_pub.publish(lane_img_msg)

        # 화면에 출력
        cv2.imshow('Lane Detection', lane_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Q pressed, shutting down...')
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()