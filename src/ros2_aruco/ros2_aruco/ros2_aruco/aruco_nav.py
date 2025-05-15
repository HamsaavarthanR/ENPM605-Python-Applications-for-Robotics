#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

# ArUco message
from ros2_aruco_interfaces.msg import ArucoMarkers

# TF2 core
from tf2_ros import Buffer, TransformListener, LookupException

# Adapter so Buffer knows how to handle PoseStamped
import tf2_geometry_msgs  
from tf2_geometry_msgs import do_transform_pose

# Standard ROS pose type
from geometry_msgs.msg import PoseStamped

# For quaternion → Euler conversion (optional)
import tf_transformations


class ArucoMapPosePrinter(Node):
    def __init__(self):
        super().__init__('aruco_map_pose_printer')

        # Set up TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to ArUco detections
        self.sub = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.on_markers,
            10
        )

    def on_markers(self, msg: ArucoMarkers):
        if not msg.marker_ids:
            self.get_logger().info("No markers detected.")
            return

        for idx, marker_id in enumerate(msg.marker_ids):
            # 1) Build a PoseStamped in the camera frame
            ps_camera = PoseStamped()
            ps_camera.header = msg.header         # stamp + frame_id
            ps_camera.pose = msg.poses[idx]       # the pose in camera coords

            try:
                # 2) Lookup the transform: map ← camera_frame
                transform = self.tf_buffer.lookup_transform(
                    'map',                          # target_frame
                    ps_camera.header.frame_id,      # source_frame (e.g. camera_link)
                    ps_camera.header.stamp,         # at the exact image time
                    timeout=Duration(seconds=1.0)
                )

                # 3) Apply it to get the pose in map
                ps_map = do_transform_pose(ps_camera, transform)

                # 4) Extract & print position + orientation
                p = ps_map.pose.position
                q = ps_map.pose.orientation
                roll, pitch, yaw = tf_transformations.euler_from_quaternion(
                    [q.x, q.y, q.z, q.w]
                )

                self.get_logger().info(
                    f"Marker {marker_id} in MAP frame:\n"
                    f"  Position → x: {p.x:.3f}, y: {p.y:.3f}, z: {p.z:.3f} (m)\n"
                    f"  Orientation (quaternion) → "
                    f"x={q.x:.3f}, y={q.y:.3f}, z={q.z:.3f}, w={q.w:.3f}\n"
                    f"  Orientation (RPY) → "
                    f"roll={roll:.2f} rad, pitch={pitch:.2f} rad, yaw={yaw:.2f} rad"
                )

            except LookupException as e:
                self.get_logger().warn(f"TF lookup failed: {e}")


def main():
    rclpy.init()
    node = ArucoMapPosePrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
