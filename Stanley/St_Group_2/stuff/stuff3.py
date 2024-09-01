import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion

class Stanley(Node):
    def __init__(self):
        super().__init__('stanley_controller')

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Parameters for the frames
        self.declare_parameter('source_frame_left', 'ego_racecar/front_left_hinge')
        self.declare_parameter('source_frame_right', 'ego_racecar/front_right_hinge')
        self.declare_parameter('reference_frame', 'map')  # Assuming 'map' is the global frame

        # Publisher
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer to periodically compute the front axle center
        self.timer = self.create_timer(0.5, self.compute_front_axle_center)

    def compute_front_axle_center(self):
        source_frame_left = self.get_parameter('source_frame_left').get_parameter_value().string_value
        source_frame_right = self.get_parameter('source_frame_right').get_parameter_value().string_value
        reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value

        try:
            # Wait for the transform to be available
            self.tf_buffer.wait_for_transform_async(reference_frame, source_frame_left, rclpy.time.Time())
            self.tf_buffer.wait_for_transform_async(reference_frame, source_frame_right, rclpy.time.Time())

            # Look up transforms
            left_transform = self.tf_buffer.lookup_transform(reference_frame, source_frame_left, rclpy.time.Time())
            right_transform = self.tf_buffer.lookup_transform(reference_frame, source_frame_right, rclpy.time.Time())

            # Calculate the center of the front axle
            center_x_relative = (left_transform.transform.translation.x + right_transform.transform.translation.x) / 2
            center_y_relative = (left_transform.transform.translation.y + right_transform.transform.translation.y) / 2
            center_z_relative = (left_transform.transform.translation.z + right_transform.transform.translation.z) / 2

            # Log the rotation and computed roll, pitch, and yaw
            quaternion = (
                left_transform.transform.rotation.x,
                left_transform.transform.rotation.y,
                left_transform.transform.rotation.z,
                left_transform.transform.rotation.w
            )
            roll, pitch, yaw = euler_from_quaternion(quaternion)

            # Log the front axle center coordinates and rotation
            self.get_logger().info(f'Front axle center coordinates (relative): x={center_x_relative}, y={center_y_relative}, z={center_z_relative}')
            self.get_logger().info(f'Front axle center rotation (quaternion): x={left_transform.transform.rotation.x}, y={left_transform.transform.rotation.y}, z={left_transform.transform.rotation.z}, w={left_transform.transform.rotation.w}')
            self.get_logger().info(f'Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}')

        except TransformException as e:
            self.get_logger().warn(f'Transform exception: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Stanley()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
