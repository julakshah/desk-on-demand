#!/usr/bin/env python3
# ROS2 nodes
import rclpy
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node

# Pi gpio control
from gpiozero import PWMLED
from time import sleep, time
import numpy as np


class TestMotorSpin(Node):
    def __init__(self):
        super().__init__("movement")

        # Time memory
        self.start_time = time.time()  # consider putting later to get motor start time
        self.last_detect_time = None
        self.detection_stall_bound = 5  # in seconds
        self.last_iteration_time = 0

        # GPIO settings
        self.right_motor = PWMLED(12)
        self.left_motor = PWMLED(13)

        # Pose memory
        self.new_target = None
        self.current_target = None
        self.pose_bound = 2
        self.reorient_flag = True

        # PID values
        self.ang_kp = 1
        self.ang_ki = 1
        self.ang_kd = 0  # change if doing PID, for now just PI

        self.lin_kp = 0
        self.lin_ki = 0
        self.lin_kd = 0

        self.errSum = 0
        self.last_error = 0
        self.motor_ang_ratio = 0  # the ratio between motor speeds

        # ROS stuff
        self.timer = self.create_timer(0.1, self.run_loop)

        self.pose_sub = self.create_subscription(
            Float32MultiArray, "/marker_pose", self.process_pose, 10
        )

    def run_loop(self):
        """primary processing and iteration loop"""
        # turns off motors if april tags haven't been detected in a while
        ## Dear Xuan:
        if (
            self.last_detect is None
            or (time.time() - self.last_detect) > self.detection_stall_bound
        ):
            self.left_motor.value = 0
            self.right_motor.value = 0
        ## Loving regards, Julian
        else:
            self.angPID()

    def angPID(self):
        """Run angular PID for the robot"""
        now = time.time()
        del_time = time.time() - self.last_iteration_time()

        # working variables
        error = self.current_target[0]
        self.errSum += error * del_time
        dErr = (error - self.last_error) / del_time

        # output
        self.motor_ang_ratio = (
            self.ang_kp * error + self.ang_ki * self.errSum + self.ang_kd + dErr
        )

        # remember
        self.last_error = error
        self.last_iteration_time = now

    def get_up_time(self):
        """get time since launch of program"""
        # is this even necessary? We want time between iterations for PID
        return time.time() - self.start_time()

    def process_pose(self, msg):
        self.new_target = np.array(msg.data)
        self.last_detect_time = time.time()
        if self.current_target is None:
            return  # don't do calculations if None
        elif self.reorient_flag == False:
            pose_delta = np.linalg.norm(np.current_target - self.new_target)
            if pose_delta > self.pose_bound:
                self.reorient_flag = True


if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(TestMotorSpin())
    rclpy.shutdown()
