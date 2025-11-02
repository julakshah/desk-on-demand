#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String
from rclpy.node import Node

class SubTest(Node):
    def __init__(self):
        super().__init__("Simple_Subscriber")
        self.sub = self.create_subscription(String,"/sample_topic",self.topic_callback,10)

    def topic_callback(self,msg):
        print(f"Received Message: {msg.data}")


if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(SubTest())
    rclpy.shutdown()