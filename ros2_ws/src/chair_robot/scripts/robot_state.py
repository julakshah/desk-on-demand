#!/usr/bin/env python3
import math
import os
import yaml
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion, TwistStamped, Vector3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from transform_helper import FrameUpdater
from pose_from_aruco import VideoProcess

class RobotState(Node):
    def __init__(self):
        super().__init__("robot_state")
        self.timer = self.create_timer(0.01,self.main_loop)
        self.name = "robot0"
        self.id = 0
        self.teleop_sub = self.create_subscription(Int32,"/use_teleop",self.use_teleop_callback,10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "cmd_vel",10)
        self.state = "follow"

        # Create TF2 frame broadcasters
        self.robot_to_world = FrameUpdater(node=self,parent=self.name,child="world",id=self.id)

        # If we're the base robot, do the target frame update
        if self.id == 0:
            self.target_to_world = FrameUpdater(node=self,parent="target",child="world",id=-1)
        
        self.vid_process = VideoProcess(node=self,use_gui=False,channel=0)


    def main_loop(self):        
        match (self.state):
            case "follow":
                pose_updates = self.vid_process.process_frame()
                for update in pose_updates:
                    camera_frame = self.name + "/camera"
                self.state_change()
            case "wait":
                pose_updates = self.vid_process.process_frame()
                self.state_change()
            case "teleop-idle":
                pass
                self.state_change()
            case "teleop-drive":
                pass
                self.state_change()
            case "stop":
                pass
                self.state_change()
            case "search":
                pose_updates = self.vid_process.process_frame()
                self.state_change()
    
    def state_change(self):
        match (self.state):
            case "follow":
                pass
            case "wait":
                pass
            case "teleop-idle":
                pass
            case "teleop-drive":
                pass
            case "stop":
                pass
            case "search":
                pass
    
    def use_teleop_callback(self,msg):
        teleop_int = msg.data
        if teleop_int < 0:
            self.state = "search"
        elif teleop_int == self.id:
            self.state = "teleop-drive"
        else:
            self.state = "teleop-idle"
    
    def publish_movement_cmd(self,linear,angular):
        cmd = TwistStamped()
        cmd.header.frame_id = self.name
        cmd.header.stamp.nanosec = self.get_clock().now().nanoseconds

        lin = Vector3(linear)
        ang = Vector3(angular)
        cmd.twist.linear = lin
        cmd.twist.angular = ang

        self.cmd_vel_pub.publish(cmd)

                   
if __name__ == "__main__":
    rclpy.init()
    robot = RobotState()
    rclpy.spin(robot)
    rclpy.shutdown()
