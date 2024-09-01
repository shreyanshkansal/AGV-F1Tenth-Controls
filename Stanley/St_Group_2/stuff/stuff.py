import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class Stanley(Node):
    def __init__(self):
        super().__init__('stanley_controller_node')
        self.subscription = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.subscription
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.drive_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.declare_parameter('target_frame', 'ego_racecar/base_link')
        self.declare_parameter('source_frame_left', 'ego_racecar/front_left_hinge')
        self.declare_parameter('source_frame_right', 'ego_racecar/front_right_hinge')

        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.current_odom = None  

        #self.timer = self.create_timer(0.5, self.timer_callback)
        #def timer_callback(self):


    def odom_callback(self, msg):
        self.current_odom = msg  

    
        if self.current_odom is None:
            self.get_logger().warn('No odom data received yet')
            return

        target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        source_frame_left = self.get_parameter('source_frame_left').get_parameter_value().string_value
        source_frame_right = self.get_parameter('source_frame_right').get_parameter_value().string_value
        
        try:
            
            left_transform = self.tf_buffer.lookup_transform(target_frame, source_frame_left, rclpy.time.Time())
            right_transform = self.tf_buffer.lookup_transform(target_frame, source_frame_right, rclpy.time.Time())
            
           
            left_x = left_transform.transform.translation.x
            left_y = left_transform.transform.translation.y
            left_z = left_transform.transform.translation.z
            
            right_x = right_transform.transform.translation.x
            right_y = right_transform.transform.translation.y
            right_z = right_transform.transform.translation.z
            
            center_x_relative = (left_x + right_x) / 2
            center_y_relative = (left_y + right_y) / 2
            center_z_relative = (left_z + right_z) / 2
            
            base_link_position = self.current_odom.pose.pose.position
            center_x_absolute = base_link_position.x + center_x_relative
            center_y_absolute = base_link_position.y + center_y_relative
            center_z_absolute = base_link_position.z + center_z_relative
            self.get_logger().info(f'Front axle center coordinates (absolute): x={center_x_absolute}, y={center_y_absolute}, z={center_z_absolute}')
            
            front_axle_center_transform = TransformStamped()
            front_axle_center_transform.header.stamp = self.get_clock().now().to_msg()
            front_axle_center_transform.header.frame_id = 'map'  
            front_axle_center_transform.child_frame_id = 'front_axle_center'
            
            front_axle_center_transform.transform.translation.x = center_x_absolute
            front_axle_center_transform.transform.translation.y = center_y_absolute
            front_axle_center_transform.transform.translation.z = center_z_absolute
            
            front_axle_center_transform.transform.rotation = self.current_odom.pose.pose.orientation  
            self.tf_broadcaster.sendTransform(front_axle_center_transform)
        
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