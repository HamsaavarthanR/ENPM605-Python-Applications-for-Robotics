#!/usr/bin/env python3
#libraries
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import math
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf2_ros import TransformBroadcaster, TransformListener, Buffer

import tf_transformations
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Class to handle the navigation based on the detected ArUco markers
class CubeNavigator(Node):
    def __init__(self, node_name):
        # Initialize the ROS2 node and its parameters
        super().__init__(node_name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self._navigation_cb_group = ReentrantCallbackGroup()
        self._tf_buffer = Buffer()  # TF buffer to store transformations
        self._tf_listener = TransformListener(self._tf_buffer, self) # TF listener to receive transformations
        self._navigator = BasicNavigator() # Basic navigation system to control robot movement
        self._sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True) # Simulation time parameter
        self.set_parameters([self._sim_time])
        self._cv_bridge = CvBridge() # Bridge to convert ROS images to OpenCV format
        self._image_subscriber = None  # Image subscriber for ArUco marker detection
        self._detected_ids = None  # Store the detected ArUco marker IDs
        self._clockwise = None # Flag for determining the direction of rotation

        # # Load 'cube2_goal' parameters to get goal positions for the robot
        self._goal_params = self.get_parameters_by_prefix('cube2_goal')
        self._cube2_goal = {}
        for key, param in self._goal_params.items():
            parts = key.split('.')
            if len(parts) != 2:
                continue
            goal_name, field = parts
            if goal_name not in self._cube2_goal:
                self._cube2_goal[goal_name] = {}
            self._cube2_goal[goal_name][field] = param.value

        # Load 'final_goal' parameters to get final goal positions for the robot
        self._final_goal_params = self.get_parameters_by_prefix('final_goal')
        self._final_goal = {}
        for key, param in self._final_goal_params.items():
            parts = key.split('.')
            if len(parts) != 2:
                continue
            goal_id, field = parts
            if goal_id not in self._final_goal:
                self._final_goal[goal_id] = {}
            self._final_goal[goal_id][field] = param.value

        self.get_logger().info("CubeNavigator initializing...")

        self._goal_1 = None

        # Goal subscription and callback setup
        self._goal1_sub = self.create_subscription(
            Pose,
            '/goal_position_1',
            self.goal1_cb,
            10,
            callback_group=self._navigation_cb_group
        )

        # Initialize timer for navigation after 5 seconds
        self._init_timer = self.create_timer(5.0, self._initialize_navigation_callback)

    def _initialize_navigation_callback(self):
        self._init_timer.cancel()
        try:
            self._navigator = BasicNavigator() # Reinitialize the navigator
            self.localize() # Set the robot's initial position

            if self._goal_1 is not None:
                x_1, y_1, ox, oy, oz, ow = self._goal_1
                self.navigate1(x_1, y_1, ox, oy, oz, ow)
            else:
                self.get_logger().warn("Waiting for goal from /goal_position_1")
                self._init_timer = self.create_timer(2.0, self._initialize_navigation_callback)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize navigation: {str(e)}")
            self._init_timer = self.create_timer(5.0, self._initialize_navigation_callback)

    def localize(self):
        # Set the robot's initial pose in the 'map' frame
        self._initial_pose = PoseStamped()
        self._initial_pose.header.frame_id = "map"
        self._initial_pose.header.stamp = self.get_clock().now().to_msg()
        self._initial_pose.pose.position.x = 0.0
        self._initial_pose.pose.position.y = 0.0
        self._initial_pose.pose.orientation.w = 1.0
        self._navigator.setInitialPose(self._initial_pose)

    def navigate1(self, x: float, y: float, ox: float, oy: float, oz: float, ow: float):
        # Navigate the robot to the first goal (Cube 1)
        self._navigator.waitUntilNav2Active()
        goal = self.create_pose_stamped(x, y, ox, oy, oz, ow)
        self._navigator.goToPose(goal)

        while not self._navigator.isTaskComplete():
            feedback = self._navigator.getFeedback()
            self.get_logger().info(f"Feedback: {feedback}")

        result = self._navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal CUBE 1 succeeded")
            if self._image_subscriber is None:
                self._image_subscriber = self.create_subscription(
                    Image,
                    '/camera/color/image_raw',
                    self.camera_image_callback1,
                    10
                )
        else:
            self.get_logger().warn(f"Goal CUBE 1 failed or canceled")

    def camera_image_callback1(self, msg: Image):
        # Callback to handle camera images after reaching Cube 1
        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Detect ArUco markers in the image
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50) # Store the detected ArUco marker IDs
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)

        # Check if any detected marker matches the cube2_goal
        if ids is not None:
            self._detected_ids = ids.flatten().tolist()
            self.get_logger().info(f"Detected ArUco marker IDs1: {self._detected_ids}")

            for goal_name, goal_data in self._cube2_goal.items():
                if goal_data.get("id") in self._detected_ids:
                    position = goal_data.get("position")
                    orientation = goal_data.get("orientation")
                    direction = goal_data.get("circular_direction")

                    self._clockwise = (direction == 0)
                    self.get_logger().info(f"Navigating to {goal_name}")
                    self.navigate2(position[0], position[1], orientation[0], orientation[1], orientation[2], orientation[3])

                    #After reaching cube 2, subscribe to the camera image again for final goal detection
                    self._image_subscriber1 = self.create_subscription(
                        Image,
                        '/camera/color/image_raw',
                        self.camera_image_callback2,
                        10
                    )                    
                    break
        else:
            self.get_logger().info("No ArUco markers detected in image.")

        # Destroy the image subscriber once the callback is complete
        if self._image_subscriber is not None:
            self.destroy_subscription(self._image_subscriber)
            self._image_subscriber = None

    def camera_image_callback2(self, msg: Image):
        # Callback to handle camera images after reaching Cube 2
        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Detect ArUco markers in the image
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)

        if ids is not None:
            self._detected_ids = (ids.flatten().tolist())
            self._detected_ids = str(self._detected_ids[0])

            self.get_logger().info(f"Detected ArUco marker IDs: {self._detected_ids}")

            # Check if detected marker matches final goal
            for goal_id, goal_data in self._final_goal.items():
                if(goal_id == self._detected_ids):
                    position = goal_data["position"]
                    orientation = goal_data["orientation"]
                    break
            # Perform a circular motion before proceeding to the final goal
            self.perform_rotation(direction=self._clockwise, radius=1, steps=16)

            self.get_logger().info(f"Navigating to FINAL goal: {position}")
            self.navigate2(position[0], position[1], orientation[0], orientation[1], orientation[2], orientation[3])
        else:
            self.get_logger().info("No final ArUco markers detected.")

        # Destroy the image subscriber once the callback is complete
        if self._image_subscriber1 is not None:
            self.destroy_subscription(self._image_subscriber1)
            self._image_subscriber1 = None

    def perform_rotation(self, direction: bool, radius=1, steps=12):
        # Perform circular motion around the detected marker
        angles_deg = [i * (360 / steps) for i in range(steps)]
        if direction:
            angles_deg.reverse()
            angles_deg = angles_deg[:-4]
        else:
            angles_deg = angles_deg[4:]
            angles_deg += [390, 420, 450, 470]

        waypoints = []
        cx, cy = 3.75, -8.50  # Center of the circular path cube 2
        # Perform circular motion by generating waypoints along a circular path with a specified radius.
        for angle_deg in angles_deg:
            rad = math.radians(angle_deg)
            x = cx + radius * math.cos(rad)
            y = cy + radius * math.sin(rad)
            yaw = rad + (-math.pi / 2 if direction else math.pi / 2)
            q = tf_transformations.quaternion_from_euler(0, 0, yaw)

            # Create waypoints for circular motion
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self._navigator.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            waypoints.append(pose)

        # Follow the waypoints for circular motion
        self._navigator.followWaypoints(waypoints)
        while not self._navigator.isTaskComplete():
            feedback = self._navigator.getFeedback()
            self.get_logger().info(f"Circular Feedback: {feedback}")
        result = self._navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Circular navigation succeeded.")
        else:
            self.get_logger().warn(f"Circular navigation failed: result={result}")

    def navigate2(self, x: float, y: float, ow: float, ox: float, oy: float, oz: float):
        # Navigate the robot to the second goal or final goal
        self._navigator.waitUntilNav2Active()
        goal = self.create_pose_stamped(x, y, ox, oy, oz, ow)
        self._navigator.goToPose(goal)
        while not self._navigator.isTaskComplete():
            feedback = self._navigator.getFeedback()
            self.get_logger().info(f"Feedback: {feedback}")
        result = self._navigator.getResult()
        return result == TaskResult.SUCCEEDED

    def create_pose_stamped(self, x: float, y: float, ox: float, oy: float, oz: float, ow: float):
        # Create a PoseStamped message for the robot's goal position
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.x = ox
        goal.pose.orientation.y = oy
        goal.pose.orientation.z = oz
        goal.pose.orientation.w = ow
        return goal

    def goal1_cb(self, msg: Pose):
        # Callback to handle incoming goals
        self._goal_1 = [msg.position.x, msg.position.y, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.get_logger().info(f"Received goal from camera: pos=({msg.position.x:.2f}, {msg.position.y:.2f}) orient=({msg.orientation.x:.2f}, {msg.orientation.y:.2f}, {msg.orientation.z:.2f}, {msg.orientation.w:.2f})")

# Class to handle ArUco marker detection and transform publishing
class CameraAruco(Node):
    def __init__(self):
        super().__init__("camera_aruco")
        self.declare_parameter("publish_tf", True)
        self._publish_tf = self.get_parameter("publish_tf").value
        self._initial_bot_pose = 2.0 # Initial bot pose

        self._camera_cb_group = ReentrantCallbackGroup()
        self._qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Subscription to ArUco markers
        self._camera_subscriber = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.camera_cb,
            qos_profile=self._qos_profile,
            callback_group=self._camera_cb_group
        )

        # Publisher to send goal position to CubeNavigator
        self._goal1_pub = self.create_publisher(
            Pose,
            '/goal_position_1',
            10,
            callback_group=self._camera_cb_group
        )

        self._aruco_marker_id = None # Store detected ArUco marker ID
        if self._publish_tf:
            self.aruco_broadcaster = TransformBroadcaster(self)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._transform_check_timer = self.create_timer(0.5, self.check_transforms)
        self.get_logger().info("CameraAruco node initialized")

    def camera_cb(self, aruco_msg: ArucoMarkers):
        # Callback to handle incoming ArUco markers
        self._aruco_marker_id = aruco_msg.marker_ids[0] # Get the first marker ID
        marker_pose = aruco_msg.poses[0] # Get the pose of the detected marker
        ##################Change 8.0 and 7.0 based on camera position######################
        x = marker_pose.position.x - 8.0 # Adjust the position based on camera distance
        dist_camera_from_cube = 1.0 # Distance of the camera from the cube
        y = marker_pose.position.y + (7.0 + dist_camera_from_cube) - self._initial_bot_pose
        z = marker_pose.position.z
        ox, oy, oz, ow = marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z, marker_pose.orientation.w

        if self._publish_tf:
            # Publish the transform for the detected marker
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = self.get_clock().now().to_msg()
            tf_stamped.header.frame_id = "map"
            tf_stamped.child_frame_id = f"aruco_marker_{self._aruco_marker_id}"
            tf_stamped.transform.translation.x = x
            tf_stamped.transform.translation.y = y
            tf_stamped.transform.translation.z = z
            tf_stamped.transform.rotation.x = ox
            tf_stamped.transform.rotation.y = oy
            tf_stamped.transform.rotation.z = oz
            tf_stamped.transform.rotation.w = ow
            self.aruco_broadcaster.sendTransform(tf_stamped)

    def check_transforms(self):
        # Periodically check for transform and update goal
        if self._aruco_marker_id is None:
            return

        marker_frame = f"aruco_marker_{self._aruco_marker_id}"
        try:
            # Look up the transform for the detected marker
            tf = self._tf_buffer.lookup_transform("map", marker_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1))

            # Publish the goal position to the CubeNavigator
            pose = Pose()
            pose.position.x = tf.transform.translation.x
            pose.position.y = tf.transform.translation.y + 1.5
            pose.position.z = tf.transform.translation.z
            pose.orientation = tf.transform.rotation

            self._goal1_pub.publish(pose)
            self.get_logger().info("Published goal pose to /goal_position_1")

        except Exception as e:
            if "extrapolation" not in str(e):
                self.get_logger().warn(f"TF lookup failed: {str(e)}")

# Main function to initialize and run the nodes
def main(args=None):
    rclpy.init(args=args)   # Initialize ROS2
    navigator_node = CubeNavigator("cube_navigator_node")  #Create CubeNavigator node
    camera_aruco_node = CameraAruco() # Create CameraAruco node
    executor = MultiThreadedExecutor()  # Executor to handle multiple nodes
    executor.add_node(camera_aruco_node)
    executor.add_node(navigator_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        navigator_node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    finally:
        navigator_node.destroy_node()
        camera_aruco_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()