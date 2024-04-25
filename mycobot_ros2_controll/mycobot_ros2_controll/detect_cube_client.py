import rclpy
from rclpy.node import Node
from mycobot_interfaces.srv import DetectionRQ
from vision_msgs.msg import Detection2DArray

class ColorDetectionClient(Node):
    def __init__(self):
        super().__init__('color_detection_client')
        self.client = self.create_client(DetectionRQ, 'color_detection')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = DetectionRQ.Request()

    def send_request(self, trigger=True):
        self.request.trigger = trigger
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            if response:
                self.process_response(response)
            else:
                self.get_logger().info('Empty response received')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

    def process_response(self, response:DetectionRQ.Response):
        if response.result:
            detections = Detection2DArray()
            detections = response.result.detections
            for detection in detections:
                color = detection.id
                centerx = detection.bbox.center.position.x
                centery = detection.bbox.center.position.y
                width = detection.bbox.size_x
                height = detection.bbox.size_y
                angle = detection.bbox.center.theta
                self.get_logger().info(f'Detected {color} at ({centerx}, {centery}) with width {width}, height {height}, and angle {angle}')
        else:
            self.get_logger().info('No detections')

def main(args=None):
    rclpy.init(args=args)
    color_detection_client = ColorDetectionClient()
    color_detection_client.send_request()
    rclpy.spin(color_detection_client)
    color_detection_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
