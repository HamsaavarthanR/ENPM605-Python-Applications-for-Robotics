#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from ros2_aruco_interfaces.msg import ArucoMarkers

# TF2
from tf2_ros import Buffer, TransformListener, LookupException
import tf2_geometry_msgs    # registers do_transform_pose
from tf2_geometry_msgs import do_transform_pose


class ArucoMapNav(Node):
    def __init__(self):
        super().__init__('aruco_map_nav')

        # --- PARAMETERS (same as before) ---
        self.declare_parameter('marker_size', 0.0625)
        self.declare_parameter('aruco_dictionary_id', 'DICT_5X5_250')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('camera_frame', '')  # optional override

        self.marker_size = self.get_parameter('marker_size').value
        dict_name = self.get_parameter('aruco_dictionary_id').value
        img_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value

        # --- TF2 SETUP ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- SUBSCRIPTIONS ---
        # 1) Camera intrinsics
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.info_cb,
            qos_profile=qos_profile_sensor_data)
        # 2) Raw images
        self.create_subscription(
            Image, img_topic, self.img_cb,
            qos_profile=qos_profile_sensor_data)

        # --- PUBLISHERS (unchanged) ---
        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)

        # --- INTERNAL STATE ---
        self.info_msg = None
        self.K = None
        self.D = None
        self.bridge = CvBridge()

        # Load ArUco dictionary
        dictionary_id = getattr(cv2.aruco, dict_name, None)
        self.aruco_dict = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_params = cv2.aruco.DetectorParameters_create()


    def info_cb(self, info: CameraInfo):
        # Grab camera intrinsics once, then unsubscribe
        self.info_msg = info
        self.K = np.array(info.k).reshape((3, 3))
        self.D = np.array(info.d)
        self.destroy_subscription(self.info_sub)
        self.get_logger().info('Camera intrinsics received.')


    def img_cb(self, img: Image):
        if self.info_msg is None:
            self.get_logger().warn('No CameraInfo yet—skipping frame.')
            return

        # Convert to OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(img, 'mono8')

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            cv_img, self.aruco_dict, parameters=self.aruco_params)

        aruco_msg = ArucoMarkers()
        pose_array = PoseArray()
        frame_id = self.camera_frame or self.info_msg.header.frame_id
        stamp = img.header.stamp

        aruco_msg.header.frame_id = frame_id
        aruco_msg.header.stamp = stamp
        pose_array.header = aruco_msg.header

        if ids is None:
            self.get_logger().info('No markers detected.')
        else:
            # Estimate pose of each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.K, self.D)

            for i, marker_id in enumerate(ids.flatten()):
                # Build a Pose in camera frame
                p = Pose()
                p.position.x = float(tvecs[i][0][0])
                p.position.y = float(tvecs[i][0][1])
                p.position.z = float(tvecs[i][0][2])

                R = cv2.Rodrigues(rvecs[i][0])[0]
                M = np.eye(4)
                M[:3, :3] = R
                q = tf_transformations.quaternion_from_matrix(M)
                p.orientation.x = q[0]
                p.orientation.y = q[1]
                p.orientation.z = q[2]
                p.orientation.w = q[3]

                # Append to original topics
                pose_array.poses.append(p)
                aruco_msg.poses.append(p)
                aruco_msg.marker_ids.append(int(marker_id))

                # --- NOW TRANSFORM INTO MAP FRAME ---
                ps_cam = PoseStamped()
                ps_cam.header.frame_id = frame_id
                ps_cam.header.stamp = stamp
                ps_cam.pose = p

                try:
                    # lookup map ← camera
                    tf_map_cam = self.tf_buffer.lookup_transform(
                        'map',
                        frame_id,
                        stamp,
                        timeout=Duration(seconds=1.0))

                    # apply it
                    ps_map = do_transform_pose(ps_cam, tf_map_cam)

                    # unpack for logging
                    x, y, z = ps_map.pose.position.x, ps_map.pose.position.y, ps_map.pose.position.z
                    qx, qy, qz, qw = (
                        ps_map.pose.orientation.x,
                        ps_map.pose.orientation.y,
                        ps_map.pose.orientation.z,
                        ps_map.pose.orientation.w,
                    )
                    roll, pitch, yaw = tf_transformations.euler_from_quaternion(
                        [qx, qy, qz, qw]
                    )

                    self.get_logger().info(
                        f"Marker {marker_id} in MAP frame:\n"
                        f"  Position → x:{x:.3f}, y:{y:.3f}, z:{z:.3f} m\n"
                        f"  Orientation (RPY) → roll:{roll:.2f}, pitch:{pitch:.2f}, yaw:{yaw:.2f}"
                    )

                except LookupException as e:
                    self.get_logger().warn(f"TF lookup failed: {e}")

        # Publish the original detections
        self.poses_pub.publish(pose_array)
        self.markers_pub.publish(aruco_msg)


def main():
    rclpy.init()
    node = ArucoMapNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
