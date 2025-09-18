import numpy as np
from typing import Tuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped ,Point, TransformStamped
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener,StaticTransformBroadcaster,TransformBroadcaster
from sensor_msgs.msg import Image

# Import your NatNetClient (adjust import to match your structure)
from natnet_client.NatNetClient import NatNetClient
import time

from scipy.spatial.transform import Rotation as R
class NatNetClientPubNode(Node):
    def __init__(self):
        super().__init__("natnet_client_pub_node")

        self.declare_parameter('client_address', '132.66.53.99')
        self.declare_parameter('server_address', '132.66.51.232')
        self.declare_parameter('use_multicast', False)
        
        self.declare_parameter('rate', 200.0)  # Hz
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('end_effector_frame', 'end_effector_link')
        self.declare_parameter('calibrate', True)
        self.declare_parameter('t_calib', [-0.0559, -0.5796, -0.3124])  # Calibration offset
        self.declare_parameter('q_calib', [0.4964, 0.5093, 0.4924, 0.5018])  # Quaternion calibration offset
        self.declare_parameter('robot_base_rb_id', 2)
        self.declare_parameter('robot_ee_rb_id', 3)
        self.declare_parameter('fixed_ee_rb_id', 1)

        self.unlabeled_marker_publisher_ = self.create_publisher(
            Marker, "natnet/unlabeled_marker_data", 10
        )  
        self.base_link_publisher_ = self.create_publisher(
            PoseStamped, "natnet/base_link_pose", 10
        )
        self.robot_ee_publisher_ = self.create_publisher(
            PoseStamped, "natnet/robot_ee_pose", 10
        )
        self.fixed_ee_publisher_ = self.create_publisher(
            PoseStamped, "natnet/fixed_ee_pose", 10
        )

        self.digit_sub = self.create_subscription(
            Image,  # Replace with your custom message type if needed
            "digit_0/image_raw",  # Adjust topic name as needed
            self.digit_callback,
            10,
        )
        # rate = self.get_parameter('rate').get_parameter_value().double_value
        # self.timer = self.create_timer(1.0 / rate, self.digit_callback)
        self.client_address = self.get_parameter('client_address').get_parameter_value().string_value
        self.server_address = self.get_parameter('server_address').get_parameter_value().string_value
        self.use_multicast = self.get_parameter('use_multicast').get_parameter_value().bool_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.robot_ee_frame = self.get_parameter('end_effector_frame').get_parameter_value().string_value
        # Calibration parameters
        self.calibrate = self.get_parameter('calibrate').get_parameter_value().bool_value
        self.calib = np.array(self.get_parameter('t_calib').get_parameter_value().double_array_value)
        self.q_calib = np.array(self.get_parameter('q_calib').get_parameter_value().double_array_value)
        self.robot_base_rb_id = self.get_parameter('robot_base_rb_id').get_parameter_value().integer_value
        self.robot_ee_rb_id = self.get_parameter('robot_ee_rb_id').get_parameter_value().integer_value
        self.fixed_ee_rb_id = self.get_parameter('fixed_ee_rb_id').get_parameter_value().integer_value
        self.get_logger().info("NatNetClientPubNode started.")
        # self.get_logger().info(f"Publishing at {rate} Hz")
        self.get_logger().info(f"Base frame: {self.base_frame}")
        self.get_logger().info(f"End effector frame: {self.robot_ee_frame}")
        self.get_logger().info(f"Calibration enabled: {self.calibrate}")
        self.get_logger().info(f"Calibration offsets: {self.calib}")
        self.get_logger().info(f"Quaternion calibration offsets: {self.q_calib}")
        self.get_logger().info(f"Robot base rigid body ID: {self.robot_base_rb_id}")
        self.get_logger().info(f"Robot end effector rigid body ID: {self.robot_ee_rb_id}")
        self.get_logger().info(f"Fixed end effector rigid body ID: {self.fixed_ee_rb_id}")


        # Initialize NatNetClient
        self.client = NatNetClient()
        self.client_is_running = False
        self.client_init()
        self.last_time = self.get_clock().now()
        
        # TF 
        self.natnet_world_frame = "natnet_world"
        self.tf_buffer = Buffer()
        # static transform broadcaster
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)
        # send static transforms for baselink->natnet world

    def calib_natnet_world(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.natnet_world_frame
        tf.transform.translation.x = float(self.calib[0])
        tf.transform.translation.y = float(self.calib[1])
        tf.transform.translation.z = float(self.calib[2])
        tf.transform.rotation.x = float(self.q_calib[0])
        tf.transform.rotation.y = float(self.q_calib[1])
        tf.transform.rotation.z = float(self.q_calib[2])
        tf.transform.rotation.w = float(self.q_calib[3])
        self.tf_broadcaster.sendTransform(tf)
        # self.get_logger().info(
        #     f"Published static TF: {self.base_frame} -> {self.natnet_world_frame} | "
        #     f"t={tf.transform.translation} q(xyzw)={tf.transform.rotation}"
        # )
    def client_init(self):
        self.client.set_client_address(self.client_address)
        self.client.set_server_address(self.server_address)
        self.client.set_use_multicast(self.use_multicast)
        # self.client.set_nat_net_version(4, 0)
        self.client.set_print_level(0)  # Set print level to 1 for more detailed output

        # client.rigid_body_listener = receive_rigid_body_frame
        self.client_is_running = self.client.run(thread_option="d")
        while not self.client_is_running:
            self.get_logger().warn("Waiting for NatNet client to start...")
            time.sleep(1)
        assert self.client_is_running, "Failed to start NatNet client."
        print("NatNet client is running.")
        time.sleep(1)
        # Check if client is properly connected
        while not self.client.connected():
            self.get_logger().warn("NatNet client is not properly connected to server.")
        else:
            self.get_logger().info("NatNet client successfully connected to server.")

    def _process_rigid_bodies_to_tf(self, mocap_data) -> Tuple[TransformStamped, TransformStamped, TransformStamped]:
        """Process rigid body data and return pose messages."""

        base_link_tf = TransformStamped()
        robot_ee_tf = TransformStamped()
        fixed_ee_tf = TransformStamped()
        for rb_id, rb in mocap_data["rigid_bodies"].items():
            # print(f"Received rigid body {rb_id} data:")
            # print(rb)
            if rb["id"] == self.robot_base_rb_id:
                # robot_base
                base_link_tf.header.stamp = self.get_clock().now().to_msg()
                base_link_tf.header.frame_id = self.natnet_world_frame
                base_link_tf.child_frame_id = 'natnet_robot_base'
                base_link_tf.transform.translation.x = rb["pose"]["position"][0]
                base_link_tf.transform.translation.y = rb["pose"]["position"][1]
                base_link_tf.transform.translation.z = rb["pose"]["position"][2]
                base_link_tf.transform.rotation.x = rb["pose"]["orientation"][0]
                base_link_tf.transform.rotation.y = rb["pose"]["orientation"][1]
                base_link_tf.transform.rotation.z = rb["pose"]["orientation"][2]
                base_link_tf.transform.rotation.w = rb["pose"]["orientation"][3]
                
                self.tf_broadcaster.sendTransform(base_link_tf)
                # self.get_logger().info(f"Base link pose: {base_link_pose}")
                
            elif rb["id"] == self.robot_ee_rb_id:
                # robot end effector
                
                robot_ee_tf.header.stamp = self.get_clock().now().to_msg()
                robot_ee_tf.header.frame_id = self.natnet_world_frame
                robot_ee_tf.child_frame_id = 'natnet_robot_ee'
                robot_ee_tf.transform.translation.x = rb["pose"]["position"][0]
                robot_ee_tf.transform.translation.y = rb["pose"]["position"][1]
                robot_ee_tf.transform.translation.z = rb["pose"]["position"][2]
                robot_ee_tf.transform.rotation.x = rb["pose"]["orientation"][0]
                robot_ee_tf.transform.rotation.y = rb["pose"]["orientation"][1]
                robot_ee_tf.transform.rotation.z = rb["pose"]["orientation"][2]
                robot_ee_tf.transform.rotation.w = rb["pose"]["orientation"][3]
                self.tf_broadcaster.sendTransform(robot_ee_tf)
                
            elif rb["id"] == self.fixed_ee_rb_id:
                # fixed end effector
                fixed_ee_tf.header.stamp = self.get_clock().now().to_msg()
                fixed_ee_tf.header.frame_id = self.natnet_world_frame
                fixed_ee_tf.child_frame_id = 'natnet_fixed_ee'
                fixed_ee_tf.transform.translation.x = rb["pose"]["position"][0]
                fixed_ee_tf.transform.translation.y = rb["pose"]["position"][1]
                fixed_ee_tf.transform.translation.z = rb["pose"]["position"][2]
                fixed_ee_tf.transform.rotation.x = rb["pose"]["orientation"][0]
                fixed_ee_tf.transform.rotation.y = rb["pose"]["orientation"][1]
                fixed_ee_tf.transform.rotation.z = rb["pose"]["orientation"][2]
                fixed_ee_tf.transform.rotation.w = rb["pose"]["orientation"][3]
                self.tf_broadcaster.sendTransform(fixed_ee_tf)

            else:
                self.get_logger().warn(f"Unknown rigid body ID: {rb['id']}")

        return base_link_tf, robot_ee_tf, fixed_ee_tf
    def _process_rigid_bodies(self, mocap_data) -> Tuple[PoseStamped, PoseStamped, PoseStamped]:
        """Process rigid body data and return pose messages."""
        base_link_pose = None
        robot_ee_pose = None
        fixed_ee_pose = None
        base_link_pose = PoseStamped()
        robot_ee_pose = PoseStamped()
        fixed_ee_pose = PoseStamped()
        for rb_id, rb in mocap_data["rigid_bodies"].items():
            # print(f"Received rigid body {rb_id} data:")
            # print(rb)
            if rb["id"] == self.robot_base_rb_id:
                # robot_base

                base_link_pose.header.stamp = self.get_clock().now().to_msg()
                base_link_pose.header.frame_id = 'world'
                base_link_pose.pose.position.x = rb["pose"]["position"][0]
                base_link_pose.pose.position.y = rb["pose"]["position"][1]
                base_link_pose.pose.position.z = rb["pose"]["position"][2]
                base_link_pose.pose.orientation.x = rb["pose"]["orientation"][0]
                base_link_pose.pose.orientation.y = rb["pose"]["orientation"][1]
                base_link_pose.pose.orientation.z = rb["pose"]["orientation"][2]
                base_link_pose.pose.orientation.w = rb["pose"]["orientation"][3]
                # self.get_logger().info(f"Base link pose: {base_link_pose}")
                
            elif rb["id"] == self.robot_ee_rb_id:
                # robot end effector

                robot_ee_pose.header.stamp = self.get_clock().now().to_msg()
                robot_ee_pose.header.frame_id = 'world'
                robot_ee_pose.pose.position.x = rb["pose"]["position"][0]
                robot_ee_pose.pose.position.y = rb["pose"]["position"][1]
                robot_ee_pose.pose.position.z = rb["pose"]["position"][2]
                robot_ee_pose.pose.orientation.x = rb["pose"]["orientation"][0]
                robot_ee_pose.pose.orientation.y = rb["pose"]["orientation"][1]
                robot_ee_pose.pose.orientation.z = rb["pose"]["orientation"][2]
                robot_ee_pose.pose.orientation.w = rb["pose"]["orientation"][3]
                
                # self.get_logger().info(f"Robot end effector pose: {robot_ee_pose}")
            elif rb["id"] == self.fixed_ee_rb_id:
                # fixed end effector
                # self.get_logger().info(f"Received fixed end effector data: {rb}")

                fixed_ee_pose.header.stamp = self.get_clock().now().to_msg()
                fixed_ee_pose.header.frame_id = 'world'
                fixed_ee_pose.pose.position.x = rb["pose"]["position"][0]
                fixed_ee_pose.pose.position.y = rb["pose"]["position"][1]
                fixed_ee_pose.pose.position.z = rb["pose"]["position"][2]
                fixed_ee_pose.pose.orientation.x = rb["pose"]["orientation"][0]
                fixed_ee_pose.pose.orientation.y = rb["pose"]["orientation"][1]
                fixed_ee_pose.pose.orientation.z = rb["pose"]["orientation"][2]
                fixed_ee_pose.pose.orientation.w = rb["pose"]["orientation"][3]
                

            else:
                self.get_logger().warn(f"Unknown rigid body ID: {rb['id']}")
        # Publish the poses
        # robot_ee_pose.pose.position.x -= base_link_pose.pose.position.x
        # robot_ee_pose.pose.position.y -= base_link_pose.pose.position.y
        # robot_ee_pose.pose.position.z -= base_link_pose.pose.position.z
        
        self.base_link_publisher_.publish(base_link_pose)
        self.robot_ee_publisher_.publish(robot_ee_pose)
        self.fixed_ee_publisher_.publish(fixed_ee_pose)
        return base_link_pose, robot_ee_pose, fixed_ee_pose

    def _apply_calibration_transform(self, input_pose, q_calib, t_calib, new_ref_frame)-> PoseStamped:
        """Apply calibration transform to convert pose from camera frame to new reference frame.
        
        Args:
            input_pose: PoseStamped message in camera frame
            q_calib: Quaternion calibration [x, y, z, w]
            t_calib: Translation calibration [x, y, z]
            new_ref_frame: Target reference frame name
            
        Returns:
            PoseStamped: Transformed pose in new reference frame
        """
        # 1) Calib transform (Camera -> Target frame)
        # scipy expects quat as [x, y, z, w]
        R_calib = R.from_quat(q_calib)      # q_calib: [qx, qy, qz, qw]
        t_calib = np.asarray(t_calib, dtype=float).reshape(3,)  # t_calib

        # 2) Read input pose (PoseStamped) expressed in camera frame
        p_in = np.array([
            input_pose.pose.position.x,
            input_pose.pose.position.y,
            input_pose.pose.position.z
        ], dtype=float)
        q_in = np.array([
            input_pose.pose.orientation.x,
            input_pose.pose.orientation.y,
            input_pose.pose.orientation.z,
            input_pose.pose.orientation.w
        ], dtype=float)

        # 3) Transform position and orientation
        p_out = R_calib.as_matrix() @ p_in + t_calib
        # Composition in scipy: (R1 * R2) applies R2 then R1
        q_out = (R_calib * R.from_quat(q_in)).as_quat()  # quat order still [x,y,z,w]

        # 4) Create output PoseStamped message
        output_pose = PoseStamped()
        output_pose.header.stamp = self.get_clock().now().to_msg()
        output_pose.header.frame_id = new_ref_frame
        output_pose.pose.position.x = p_out[0]
        output_pose.pose.position.y = p_out[1]
        output_pose.pose.position.z = p_out[2]
        output_pose.pose.orientation.x = q_out[0]
        output_pose.pose.orientation.y = q_out[1]
        output_pose.pose.orientation.z = q_out[2]
        output_pose.pose.orientation.w = q_out[3]
        
        return output_pose

    def digit_callback(self, msg: Image):
        """for the nat net 
        robot_x =  -natnet[0]
        robot_y = natnet[2]
        robot_z = natnet[1]
        """
        self.calib_natnet_world()
        now_time = msg.header.stamp
        # if (now_time - self.last_time).nanoseconds > 1e6:  # Avoid too frequent calls
        # self.get_logger().info(f"Digit callback called at {now_time.seconds_nanoseconds()[0]}.{now_time.seconds_nanoseconds()[1]}")
        self.last_time = now_time
        # Check if we have frame data available
        if self.client.current_frame_data is None:
            # self.get_logger().warn("No frame data available from NatNet client.")
            return

        mocap_data = self.client.get_structured_mocap_data()
        if mocap_data is None:
            # self.get_logger().warn("No mocap data received.")
            return
        
        base_link_pose, robot_ee_pose, fixed_ee_pose = self._process_rigid_bodies_to_tf(mocap_data)
        base_link_pose, robot_ee_pose, fixed_ee_pose = self._process_rigid_bodies(mocap_data)
        
        # check for new calibration parameters
        calib = np.array(self.get_parameter('t_calib').get_parameter_value().double_array_value)
        q_calib = np.array(self.get_parameter('q_calib').get_parameter_value().double_array_value)
        calibrate = self.get_parameter('calibrate').get_parameter_value().bool_value
        change_detected = False
        
        if calibrate != self.calibrate:
            self.get_logger().info(f"Calibration enabled changed: original {self.calibrate} new {calibrate}")
            self.calibrate = calibrate
            change_detected = True
            
        if not np.allclose(calib, self.calib):
            self.get_logger().info(f"Calibration offsets changed: original {calib} new {self.calib}")
            self.calib = calib
        if not np.allclose(q_calib, self.q_calib):
            self.get_logger().info(f"Quaternion calibration offsets changed: original {q_calib} new {self.q_calib}")
            self.q_calib = q_calib
        # Publish unlabeled markers
        if "unlabeled_markers" in mocap_data:
            marker_msg = Marker()
            marker_msg.header.stamp = self.get_clock().now().to_msg()
            marker_msg.header.frame_id = self.natnet_world_frame  
            
            marker_msg.type = Marker.POINTS
            marker_msg.action = Marker.ADD
            marker_msg.scale.x = 0.01  # point width in meters
            marker_msg.scale.y = 0.01  # point height in meters
            marker_msg.color.a = 1.0
            marker_msg.color.r = 1.0
            marker_msg.color.g = 0.0
            marker_msg.color.b = 0.0
            marker_msg.lifetime = Duration(sec=0)  # 0 means forever
            points_list = []
            for marker in mocap_data["labeled_markers"]['unlabeled']:
                p = Point()
                p.x, p.y, p.z = marker["position"]

                points_list.append(p)
            marker_msg.points = points_list

            self.unlabeled_marker_publisher_.publish(marker_msg)
            # self.get_logger().info(f"Published {len(marker_msg.points)} unlabeled markers")


def main(args=None):
    rclpy.init(args=args)
    node = NatNetClientPubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
