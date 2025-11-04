#!/usr/bin/env python3
# ROS2 nodes
import rclpy
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node

# Pi gpio control
from gpiozero import PWMLED
from time import sleep
import numpy as np

class TestMotorSpin(Node):
    def __init__(self):
        super().__init__('movement')

        self.motor1 = PWMLED(12) #right one (for now)
        self.motor2 = PWMLED(13) #left one (for now)

        self.timer = self.create_timer(0.1, self.run_loop)


        self.new_target = None
        self.current_target = None
        self.pose_sub = self.create_subscription(Float32MultiArray, "/marker_pose", self.process_pose, 10)
        self.pose_bound = 2
        self.reorient_flag = True


    def run_loop(self):
        if self.new_target is not None:
            y = self.new_target[0] 
            if y < 0:
                self.motor1.value = 0
                self.motor2.value = .4
            elif y > 0: 
                self.motor1.value = .4
                self.motor2.value = 0
            else:
                self.motor1.value = 0
                self.motor2.value = 0

    def process_pose(self, msg):
        self.new_target = np.array(msg.data)
        if self.current_target is None:
            return #don't do calculations if None
        elif self.reorient_flag == False:
            pose_delta = np.linalg.norm(np.current_target - self.new_target)
            if pose_delta > self.pose_bound:
                self.reorient_flag = True

        

if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(TestMotorSpin())
    rclpy.shutdown()
