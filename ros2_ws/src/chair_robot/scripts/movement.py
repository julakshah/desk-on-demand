#!/usr/bin/env python3
# ROS2 nodes
import rclpy
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node

# Pi gpio control
from gpiozero import PWMLED
from time import sleep
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class TestMotorSpin(Node):
    def __init__(self):
        super().__init__('movement')

        self.motor1 = PWMLED(12) #right one (for now)
        self.motor2 = PWMLED(13) #left one (for now)

        self.timer = self.create_timer(0.1, self.run_loop)


        self.new_target = None
        self.current_target = None
        self.pose_sub = self.create_subscription(Float32MultiArray, "/marker_pose", self.process_pose, 10)
        self.cmd_vel_sub = self.create_subscription(Twist,"/cmd_vel",self.process_twist,10)
        self.use_teleop_sub = self.create_subscription(Int32,"/use_teleop",self.process_teleop,10)
        self.pose_bound = 2
        self.id = 0 # id of robot
        self.teleop_state = 0 # id of robot controlled by teleop
        self.state = "stop"
        self.reorient_flag = True

    def run_loop(self):
        match self.state:
            case "stop":
                self.drive_raw(0,0)
                return
            case "teleop":
                return
            case _:
                if self.new_target is not None:
                    y = self.new_target[0] 
                    if y < 0:
                        self.drive_raw(0,.4)
                    elif y > 0: 
                        self.drive_raw(.4,0)
                    else:
                        self.drive_raw(0,0)

    def drive_raw(self, m1, m2):
        max_percent = 0.8
        m1 = min(max(0.0,m1),max_percent)
        m2 = min(max(0.0,m2),max_percent)
        self.motor1.value = m1
        self.motor2.value = m2

    def process_twist(self, msg: Twist):
        # Change for actual units / something parameterizable
        ang = msg.angular.z
        lin = msg.linear.x
        if not (self.state == "stop" or self.state == "teleop"):
            self.drive_raw(lin + ang, lin - ang)

    def process_teleop(self, msg: Int32):
        # Update state --- -1 is auto, teleop if equal to my id, stop if not
        val = msg.data
        self.teleop_state = val
        if val == self.id:
            self.state = "teleop"
        elif val == -1:
            self.state = "drive"
        else:
            self.state = "stop"

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
