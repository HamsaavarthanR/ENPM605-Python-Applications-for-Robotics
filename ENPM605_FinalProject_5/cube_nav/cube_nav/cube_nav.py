from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import rclpy.duration
from rclpy.parameter import Parameter
from rclpy.node import Node
import tf_transformations

from rclpy.callback_groups import ReentrantCallbackGroup
# from geometry_msgs.msg import Pose

# packages for reading aruco markers
from ros2_aruco_interfaces.msg import ArucoMarkers
# import tf2_ros
# import tf2_geometry_msgs
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, HistoryPolicy
import numpy as np




class CubeNavigator(Node):
    """
    Node to handle navigation through waypoints:
    Waypoint 1: 1m left to cube 1 (Pose obtained from camera-aruco_marker transform)
    Waypoint 2: 1m left to cube 2 (Pose obtained from Parameters file corresponding to marker_id: on cube 1 )
    Waypoint 3: Final goal (Pose obtained from Parameters file corresponding to marker_id: on cube 2)
    """

    def __init__(self, node_name):
        super().__init__(node_name)
        
        # Declare paramters
        
        # Get paramters
        
        # Callback group
        self._navigation_cb_group = ReentrantCallbackGroup()

        # Since we are using Gazebo, we need to set the use_sim_time parameter to True
        self._sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([self._sim_time])
        
        self.get_logger().info("Navigation demo started, waiting for Nav2...")
        
        # Goal Positions
        self._goal_1 = None
        
        # Create subscriber to get goal 1 position from camera TF frame
        self._goal1_sub = self.create_subscription(Pose, 
                                                   '/goal_position_1',
                                                   self.goal1_cb,
                                                   10,
                                                   callback_group=self._navigation_cb_group)
        
        
        # Create a timer that calls initialize_navigation once after 5 seconds
        # self._init_timer = self.create_timer(5.0, self._initialize_navigation_callback)
        
    
    
    def _initialize_navigation_callback(self):
        # Cancel the timer so this only runs once
        self._init_timer.cancel()
        
        try:
            # Navigator
            self._navigator = BasicNavigator()
            
            # Set the initial pose of the robot
            self.localize()
            
            # Get Goal 1 Position
            if self._goal_1 is not None:
                x_1 = self._goal_1[0]
                y_1 = self._goal_1[1]
                
                self.navigate(5.0, 5.0)
            
            # Follow the waypoints
            # self.follow_waypoints()
            
            self.get_logger().info("Navigation initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize navigation: {str(e)}")
            # Try again after a delay by creating a new timer
            self._init_timer = self.create_timer(5.0, self._initialize_navigation_callback)
        
    def localize(self):
        """
        Set the initial pose of the robot.
        """
        # Set the initial pose of the robot
        self._initial_pose = PoseStamped()
        self._initial_pose.header.frame_id = "map"
        self._initial_pose.header.stamp = self._navigator.get_clock().now().to_msg()
        self._initial_pose.pose.position.x = 0.0
        self._initial_pose.pose.position.y = 0.0
        self._initial_pose.pose.position.z = 0.0
        self._initial_pose.pose.orientation.x = 0.0
        self._initial_pose.pose.orientation.y = 0.0
        self._initial_pose.pose.orientation.z = 0.0
        self._initial_pose.pose.orientation.w = 1.0
        self._navigator.setInitialPose(self._initial_pose)
        
    def navigate(self, x: float, y: float):
        """
        Navigate the robot to the goal (x, y).
        """
        self._navigator.waitUntilNav2Active() # Wait until Nav2 is active
        
        goal = self.create_pose_stamped(x, y, 0.0)
        
        self._navigator.goToPose(goal)
        while not self._navigator.isTaskComplete():
            feedback = self._navigator.getFeedback()
            self.get_logger().info(f"Feedback: {feedback}")
        
        result = self._navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().info("Goal failed!")
            
    def follow_waypoints(self):
        self._navigator.waitUntilNav2Active()  # Wait until Nav2 is active

        pose1 = self.create_pose_stamped(0.0, -3.0, 0.0)
        pose2 = self.create_pose_stamped(0.0, -5.0, 0.0)
        pose3 = self.create_pose_stamped(3.0, -7.0, 0.0)
        waypoints = [pose1, pose2, pose3]
        self._navigator.followWaypoints(waypoints)
        
        while not self._navigator.isTaskComplete():
            feedback = self._navigator.getFeedback()
            self.get_logger().info(f"Feedback: {feedback}")

        result = self._navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().info("Goal failed!")

    def create_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        """
        Create a PoseStamped message.
        """
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self._navigator.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        self._initial_pose.pose.orientation.x = q_x
        self._initial_pose.pose.orientation.y = q_y
        self._initial_pose.pose.orientation.z = q_z
        self._initial_pose.pose.orientation.w = q_w

        goal.pose.orientation.x = q_x
        goal.pose.orientation.y = q_y
        goal.pose.orientation.z = q_z
        goal.pose.orientation.w = q_w
        return goal
    
    def goal1_cb(self, msg: Pose):
        self.get_logger().info(f"{msg.position}")
    
    

## ------------------------------------------------------------------------------------------------ ##

# Node to Broadcast transform betweeen aruco detected on camera frame

class CameraAruco(Node):
    """
    Node to Broadcast transform betweeen aruco detected on camera frame
    Transform the pose of the detected marker into '/map' coordinate frame
    Calculate the Goal Position 1: 1m left to cube 1
    Publish the resultant goal position to '/goal_position_1' topic
    """

    def __init__(self):
        super().__init__("camera_aruco")
        
        # Declare paramters
        self.declare_parameter("publish_tf", True)
        
        # Get paramters
        self._publish_tf = self.get_parameter("publish_tf").value
        
        # Callback group
        self._camera_cb_group = ReentrantCallbackGroup()
        
        # Subscribe to the camera image topic with a reliable QoS profile
        self._qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # Create subcriber to obtain pose of teh aruco marker in camera frame
        self._camera_subscriber = self.create_subscription(ArucoMarkers, 
                                                           '/aruco_markers', 
                                                           self.camera_cb, 
                                                           callback_group=self._camera_cb_group,
                                                           qos_profile=self._qos_profile)
        
        
        # Keep track of detected marker ID and camera base frame
        self._camera_frame = None
        self._aruco_marker_id = None
        
        # Create object to store pose of aruco marker detected in the camera 
        # w.r.t '/map' frame --> goal 1 position
        self._aruco_goal1_pose = None

        
        # Camera pose in world '/map' frame
        # self._camera_wrt_map = None
        
        # Create publisher to update Goal 1 pose w.r.t map in '/goal_position_1' topic
        self._goal1_pub = self.create_publisher(Pose,
                                                '/goal_position_1',
                                                10,
                                                callback_group=self._camera_cb_group)
        
        # TF Broadcaster for aruco marker w.r.t. camera frame
        if self._publish_tf:
            # self.camera_broadcaster = TransformBroadcaster(self)
            self.aruco_broadcaster = TransformBroadcaster(self)
            
    
        # Initialize TF buffer and listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        # Create a timer for periodically checking transforms
        self._transform_check_timer = self.create_timer(0.5, self.check_transforms)
        
        self.get_logger().info("CameraAruco node initialised and ready")
        
        
    
    
    def camera_cb(self, aruco_msg: ArucoMarkers):
        # Retrieve message and obtain aurco marker id, pose w.r.t camera frame
        camera_frame = aruco_msg.header.frame_id
        marker_id = aruco_msg.marker_ids
        
        # Update camera, aruco marker ids
        self._camera_frame = camera_frame
        self._aruco_marker_id = marker_id[0]
        
        
        # First, camera tf wrt map
        # Get camera pose w.r.t '/map' frame
        # <pose>-8 7 0.25 0 0 1.57</pose>
        
        # Obtain aruco marker pose w.r.t '/map' frame
        marker_pose = aruco_msg.poses[0]
        x = marker_pose.position.x - 8.0
        y = marker_pose.position.y + 5.75
        z = marker_pose.position.z
        orn_x = marker_pose.orientation.x
        orn_y = marker_pose.orientation.y
        orn_z = marker_pose.orientation.z
        orn_w = marker_pose.orientation.w
        
        # self.get_logger().info(f"Camera id: {camera_frame}\n--->Pose of aruco marker [{marker_id[0]}]: [{x}, {y}, {z}]")
        
        # Creates TF Broadcaster
        if self._publish_tf:
            # Create Transform Message
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = self.get_clock().now().to_msg()
            # Use frame id from camera message
            tf_stamped.header.frame_id = "map"
            tf_stamped.child_frame_id = f"aruco_marker_{self._aruco_marker_id}"
            
            # Translation (Position)
            tf_stamped.transform.translation.x = x
            tf_stamped.transform.translation.y = y
            tf_stamped.transform.translation.z = z
            
            # Rotation (Orientation)
            tf_stamped.transform.rotation.x = orn_x
            tf_stamped.transform.rotation.y = orn_y
            tf_stamped.transform.rotation.z = orn_z
            tf_stamped.transform.rotation.w = orn_w
            
            # Broadcast the transform
            # self.camera_broadcaster.sendTransform(self._camera_wrt_map)
            self.aruco_broadcaster.sendTransform(tf_stamped)
            # self.get_logger().info("TF Broadcast Successfull!!")
            
    def check_transforms(self):
        """
            Periodically check if the transform bw camera and detected aruco marker is available
            Obtain tranform of aruco marker in '/map' frame
        """
        
        # Skip if no markers have been detected 
        if self._aruco_marker_id is None:
            return
        
        # If the marker is detected, obtain transform of aruco marker w.r.t. map frame
        target_base_frame = "map"
        marker_frame = f"aruco_marker_{self._aruco_marker_id}"
        
        try:
            # self.get_logger().info("Check transform called")
            
            aruco_map_tf = self._tf_buffer.lookup_transform(
                target_base_frame,
                marker_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1)
            )
            
            # Obtain Aruco Marker Pose w.r.t '/map' frame
            # and Create Pose Message 
            # to update 'self._aruco_goal1_pose' to publish calculated goal location wrt world
            self._aruco_goal1_pose = Pose()
            self._aruco_goal1_pose.position.x = aruco_map_tf.transform.translation.x
            self._aruco_goal1_pose.position.y = aruco_map_tf.transform.translation.y + 1.5 # (cube size + 1m)
            self._aruco_goal1_pose.position.z = aruco_map_tf.transform.translation.z
            self._aruco_goal1_pose.orientation.x = aruco_map_tf.transform.rotation.x
            self._aruco_goal1_pose.orientation.y = aruco_map_tf.transform.rotation.x
            self._aruco_goal1_pose.orientation.z = aruco_map_tf.transform.rotation.x
            self._aruco_goal1_pose.orientation.w = aruco_map_tf.transform.rotation.w
            
            self._goal1_pub.publish(self._aruco_goal1_pose)
            # self.get_logger().info("Message published")
                    
            
        except Exception as e:
            # If transform isn't available yet or has expired, don't report an error
            if "lookup would require extrapolation" not in str(e):
                self.get_logger().debug(
                    f"Cannot look up transform for marker {self._aruco_marker_id}: {e}"
                )    
            
            
                    
            
            
            
            
            
## ------------------------------------------------------------------------------------------------ ##
            
    


def main(args=None):
    rclpy.init(args=args)
    navigator_node = CubeNavigator("cube_navigator_node")
    camera_aruco_node = CameraAruco()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    
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
        