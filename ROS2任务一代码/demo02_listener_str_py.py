#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    订阅者：订阅 Num 消息，计算 num1 + num2 并打印
    对应话题：/num_topic
"""

import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Num

class ListenerNum(Node):
    def __init__(self):
        super().__init__("demo02_listener_str_py")
        self.subscription = self.create_subscription(
            Num,
            "num_topic",
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        sum_result = msg.num1 + msg.num2
        self.get_logger().info(
            f"订阅: num1 = {msg.num1}, num2 = {msg.num2}, 和 = {sum_result}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNum()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()