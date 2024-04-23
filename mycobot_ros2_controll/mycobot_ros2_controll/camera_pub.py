import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

bridge = CvBridge()
class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher_node")
        self.publisher_ = self.create_publisher(Image, "/camera/image_raw", 10)
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            sys.exit(1)

    def timer_callback(self):
        ret, frame = self.cap.read()
        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
        ros_image.header.stamp = self.get_clock().now().to_msg()  # 현재 시간으로 설정
        ros_image.header.frame_id = "camera_frame"  # 카메라의 광학 프레임 ID로 설정
        self.publisher_.publish(ros_image)
        
    def destroy_node(self):
        super().destroy_node()
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()