#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import time


class Executor(Node):
    def __init__(self):
        super().__init__("executor_node")
        self.cb_gourp_ = ReentrantCallbackGroup()
        self.timer1_ = self.create_timer(1.0, self.timer_cb, self.cb_gourp_)
        self.counter = 0
        
    def timer_cb(self):
        self.counter += 1
        self.get_logger().info(f"{self.counter}")
        time.sleep(2.0)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = Executor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()