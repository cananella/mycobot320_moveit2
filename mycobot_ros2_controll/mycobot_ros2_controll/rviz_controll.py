import time
import math
import rclpy
from pymycobot.mycobot import MyCobot
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Controll_Subscriber(Node):
    def __init__(self):
        super().__init__("controll_rviz")
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.listener_callback,
            10
        )
        self.subscription

        self.mc = MyCobot("/dev/ttyACM0", 115200)
        time.sleep(0.05)
        self.mc.set_free_mode(1)
        self.mc.set_gripper_mode(0)
        self.mc.set_gripper_calibration()
        self.mc.init_eletric_gripper()
        time.sleep(2)

        self.grippercurrVelue=0
        self.grippernextVelue=0





    def listener_callback(self, msg: JointState):
        data_list = []
        joint_name=msg.name
        joint1=joint_name.index('joint2_to_joint1')
        joint2=joint_name.index('joint3_to_joint2')
        joint3=joint_name.index('joint4_to_joint3')
        joint4=joint_name.index('joint5_to_joint4')
        joint5=joint_name.index('joint6_to_joint5')
        joint6=joint_name.index('joint6output_to_joint6')
        gripperjoint=joint_name.index('right_inside_joint')

        for idx, value in enumerate(msg.position):
            radians_to_angles = round(math.degrees(value), 2)
            if idx==joint1 or idx == joint5 or idx == joint6:
                radians_to_angles = -radians_to_angles
            if idx == gripperjoint:
                data_list.append(radians_to_angles)
            else:
                angles_to_digit= int((radians_to_angles/180)*2048+2048)
                data_list.append(angles_to_digit)


        # print(data_list)
        
        arm_data=[data_list[joint1],data_list[joint2],data_list[joint3],data_list[joint4],data_list[joint5],data_list[joint6]]

        self.grippernextVelue=int(-data_list[gripperjoint]/46*800+2048)
        
        # print('joint angles: {}'.format(arm_data))
        # print('gripper angles: {} <= {}'.format(self.grippernextVelue, self.grippercurrVelue))

        if self.grippernextVelue == self.grippercurrVelue:
            self.mc.set_encoders(arm_data, 50)
            time.sleep(0.1)


        else :
            self.mc.set_encoder(7,self.grippernextVelue,50)
            time.sleep(2)
            self.grippercurrVelue=self.grippernextVelue


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Controll_Subscriber()
    rclpy.spin(slider_subscriber)

    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
