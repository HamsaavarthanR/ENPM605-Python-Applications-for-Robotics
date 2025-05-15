#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import tf_transformations


class GoToFixedPose(Node):
    def __init__(self):
        super().__init__('go_to_fixed_pose')
        # Use sim time in Gazebo
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.get_logger().info('Waiting 5s for Nav2 to come up...')
        # Delay so that map_server & all Nav2 action servers are ready
        self._timer = self.create_timer(5.0, self._on_timer)

    def _on_timer(self):
        # run only once
        self._timer.cancel()

        # Create navigator
        self.navigator = BasicNavigator()

        # 1) Set initial pose at (0,0) facing +X
        init = PoseStamped()
        init.header.frame_id = 'map'
        init.header.stamp = self.navigator.get_clock().now().to_msg()
        init.pose.position.x = 0.0
        init.pose.position.y = 0.0
        init.pose.orientation.x = 0.0
        init.pose.orientation.y = 0.0
        init.pose.orientation.z = 0.0
        init.pose.orientation.w = 1.0
        self.navigator.setInitialPose(init)
        self.get_logger().info('Initial pose set to (0,0,0)')

        # 2) Wait for Nav2 servers
        self.navigator.waitUntilNav2Active()

        # 3) Send to (2.5, -6.5) with quaternion (x=0,y=0,z=0,w=1)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = 2.5
        goal.pose.position.y = -6.5 - 2

        # identity quaternion = no rotation
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.get_logger().info('Sending goal → x:2.5, y:-6.5, quaternion=(0,0,0,1)')
        self.navigator.goToPose(goal)

        # 4) Monitor progress
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self.get_logger().info(f'Feedback: {feedback}')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('✅ Goal reached!')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('⚠️ Goal canceled')
        else:
            self.get_logger().error('❌ Goal failed')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GoToFixedPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
