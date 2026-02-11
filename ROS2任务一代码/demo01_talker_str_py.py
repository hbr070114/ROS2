#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    发布者：发布自定义 Num 消息（包含 num1、num2）
    对应话题：/num_topic
"""

import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Num  # 导入自定义消息

class TalkerNum(Node):
    def __init__(self):
        super().__init__("demo01_talker_str_py")  # 节点名
        self.publisher_ = self.create_publisher(Num, "num_topic", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = Num()
        msg.num1 = 10 + self.count
        msg.num2 = 20 + self.count
        self.publisher_.publish(msg)
        self.get_logger().info(f"发布: num1 = {msg.num1}, num2 = {msg.num2}")
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNum()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()