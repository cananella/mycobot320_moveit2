import rclpy
from rclpy.node import Node
from mycobot_interfaces.srv import DetectionRQ
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import cv2
import numpy as np
import yaml
import threading
import sys

from pathlib import Path

# 다른 import 문들은 동일하게 유지

class ColorDetectionService(Node):
    def __init__(self,imgshow_flag=False):
        super().__init__('color_detection_service')
        self.srv = self.create_service(DetectionRQ, 'color_detection', self.detect_callback)
        self.imgshow_flag = imgshow_flag
        self._package_path = Path(get_package_share_directory('mycobot_ros2_controll'))
        self.file_path = str(self._package_path)+'/config/example.yaml'
        self.yaml_data = self.read_yaml_file(self.file_path)

        self.bridge = CvBridge()
        

        self.cap = cv2.VideoCapture(2)
        self.width =640
        self.height =480
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)


        self.armforcesX = self.yaml_data.get('robot_focus').get('x')
        self.armforcesY = self.yaml_data.get('robot_focus').get('y')
        self.detect_data=[]
        blue_high_h =self.yaml_data.get('blue').get('high_h')
        blue_low_h  =self.yaml_data.get('blue').get('low_h')
        blue_high_s =self.yaml_data.get('blue').get('high_s')
        blue_low_s  =self.yaml_data.get('blue').get('low_s')
        blue_high_v =self.yaml_data.get('blue').get('high_v')
        blue_low_v  =self.yaml_data.get('blue').get('low_v')

        green_high_h =self.yaml_data.get('green').get('high_h')
        green_low_h  =self.yaml_data.get('green').get('low_h')
        green_high_s =self.yaml_data.get('green').get('high_s')
        green_low_s  =self.yaml_data.get('green').get('low_s')
        green_high_v =self.yaml_data.get('green').get('high_v')
        green_low_v  =self.yaml_data.get('green').get('low_v')

        orange_high_h =self.yaml_data.get('orange').get('high_h')
        orange_low_h  =self.yaml_data.get('orange').get('low_h')
        orange_high_s =self.yaml_data.get('orange').get('high_s')
        orange_low_s  =self.yaml_data.get('orange').get('low_s')
        orange_high_v =self.yaml_data.get('orange').get('high_v')
        orange_low_v  =self.yaml_data.get('orange').get('low_v')

        purple_high_h =self.yaml_data.get('purple').get('high_h')
        purple_low_h  =self.yaml_data.get('purple').get('low_h')
        purple_high_s =self.yaml_data.get('purple').get('high_s')
        purple_low_s  =self.yaml_data.get('purple').get('low_s')
        purple_high_v =self.yaml_data.get('purple').get('high_v')
        purple_low_v  =self.yaml_data.get('purple').get('low_v')

        # 색상 범위 지정
        self.lower_orange = np.array([orange_low_h, orange_low_s, orange_low_v])
        self.upper_orange = np.array([orange_high_h, orange_high_s, orange_high_v])
        self.lower_blue   = np.array([blue_low_h, blue_low_s, blue_low_v])
        self.upper_blue = np.array([blue_high_h, blue_high_s, blue_high_v])
        self.lower_green = np.array([green_low_h, green_low_s, green_low_v])
        self.upper_green = np.array([green_high_h, green_high_s, green_high_v])
        self.lower_purple = np.array([purple_low_h, purple_low_s, purple_low_v])
        self.upper_purple = np.array([purple_high_h, purple_high_s, purple_high_v])

        if self.imgshow_flag:
            _,self.frame=self.cap.read()
            imgshow_thread=threading.Thread(target=self.imgShow)
            imgshow_thread.start()

    def detect_callback(self, request:DetectionRQ.Request, response:DetectionRQ.Response):
        # 여기서 색상 감지 로직을 처리하고, 결과를 response에 채워서 반환합니다.
        # request에는 필요한 요청 정보가 포함될 수 있습니다.
        # response에는 처리 결과를 담아서 클라이언트에게 반환합니다.
        if request.trigger:
            response.result = self.detect()  # detect 메서드는 Detection2DArray 메시지를 반환하도록 작성되어야 합니다.
            print("Servise Request")
        return response

    def detect(self):
        # The detection logic here is simplified for brevity
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to read frame from camera')
            return

        ## detecting process
        frame = cv2.flip(frame,0)
        frame = cv2.flip(frame,1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv,(3,3),0)

        mask_orange = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask_purple = cv2.inRange(hsv, self.lower_purple, self.upper_purple)

        orange_contours, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        purple_contours, _ = cv2.findContours(mask_purple, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_list=[orange_contours,blue_contours,green_contours,purple_contours]
        color_list=['orange','blue','green','purple']

        self.detect_data.clear()
        contourarea_threshold=100

        for idx,contour_data in enumerate(contours_list):
            for contour in contour_data:
                epsilon = 0.05 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                area = cv2.contourArea(approx)
                if area > contourarea_threshold:  # 임계값을 조절하여 작은 노이즈를 제거
                    # 컨투어의 중심 찾기
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int64(box)
                    # 중심점, 너비, 높이, 각도 추출
                    (x, y), (box_width, box_height), angle = rect
                    center = (int(x), int(y))
                    bb_width = int(box_width)
                    bb_height = int(box_height)
                    self.detect_data.append([color_list[idx],center[0],center[1],bb_width,bb_height,angle])
        
        # Construct Detection2DArray message
        detections = Detection2DArray()
        detections.detections = [self.create_detection(color, centerx, centery, width, height, angle)
                                 for color, centerx, centery, width, height, angle in self.detect_data]
        if self.imgshow_flag:
            self.frame=frame

        return detections

    def create_detection(self, color, centerx, centery, width, height, angle):
        detection = Detection2D()
        detection.id=color
        detection.bbox.center.position.x = float(centerx)
        detection.bbox.center.position.y = float(centery)
        detection.bbox.center.theta = angle
        detection.bbox.size_x = float(width)
        detection.bbox.size_y = float(height)

        return detection

    def read_yaml_file(self, file_path):
        with open(file_path, 'r') as yaml_file:
            data = yaml.safe_load(yaml_file)
        return data

    def imgShow(self):
        while True:
            _,frame=self.cap.read()
            frame = cv2.flip(frame,0)
            frame = cv2.flip(frame,1)
            for color,cX,cY,_,_,_ in self.detect_data:
                # 중심점에 점 그리기
                cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                # 중심점 좌표 출력
                transformX=cX-self.armforcesX
                transformY=self.armforcesY-cY
                cv2.putText(frame, f'{color} ({transformX}, {transformY})', (cX - 50, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            frame=cv2.line(frame,(self.armforcesX,0),(self.armforcesX,self.height),(0,255,0),1)
            frame=cv2.line(frame,(0,self.armforcesY),(self.width,self.armforcesY),(0,255,0),1)
            frame=cv2.circle(frame,(self.armforcesX,self.armforcesY),2,(0,0,255),-1)
            # 결과 출력
            cv2.imshow('Detection', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

######################################################################################################################

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) > 1 and sys.argv[1] == 'True':
        imgshow_flag = True
    else:
        imgshow_flag = False
    color_detection_service = ColorDetectionService(imgshow_flag)
    rclpy.spin(color_detection_service)
    color_detection_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()