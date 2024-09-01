import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped, Twist
from ackermann_msgs.msg import AckermannDriveStamped
from tf_transformations import euler_from_quaternion
import numpy as np
import pandas as pd
import math 
import csv

class Stanley(Node):
    def __init__(self):
        super().__init__('stanley_controller')

        self.k= 0.1
        self.ks = 2.0
        self.vel = 0.5
        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.declare_parameter('left', 'ego_racecar/front_left_hinge')
        self.declare_parameter('right', 'ego_racecar/front_right_hinge')
        self.declare_parameter('map', 'map')

        self.wayfile = '/home/goyal/sim_ws/src/my_stanley_controller/resource/waypoints.csv'
        self.waypoints = self.load_waypoints()
        self.current_waypoint_index = 0
        # Publisher
        self.drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        # Timer
        self.timer = self.create_timer(0.1, self.stanley)

    def load_waypoints(self):
        """ Load waypoints from a CSV file efficiently """
        waypoints = pd.read_csv(self.wayfile, header=None).values
        return waypoints

    def stanley(self):
        left = self.get_parameter('left').get_parameter_value().string_value
        right = self.get_parameter('right').get_parameter_value().string_value
        map = self.get_parameter('map').get_parameter_value().string_value

        try:
            # Wait for the transform to be available
            self.tf_buffer.wait_for_transform_async(map, left, rclpy.time.Time())
            self.tf_buffer.wait_for_transform_async(map, right, rclpy.time.Time())

            # Look up transforms
            lefty = self.tf_buffer.lookup_transform(map, left, rclpy.time.Time())
            righty = self.tf_buffer.lookup_transform(map, right, rclpy.time.Time())

            # Calculate the center of the front axle
            _x = (lefty.transform.translation.x + righty.transform.translation.x) / 2
            _y = (lefty.transform.translation.y + righty.transform.translation.y) / 2
            _z = (lefty.transform.translation.z + righty.transform.translation.z) / 2


            # Broadcast the transform for the front axle center
            at = TransformStamped()
            at.header.stamp = self.get_clock().now().to_msg()
            at.header.frame_id = map
            at.child_frame_id = 'axle'

            at.transform.translation.x = _x
            at.transform.translation.y = _y
            at.transform.translation.z = _z

            # Use the orientation from one of the hinge frames, for instance, the left hinge
            at.transform.rotation = lefty.transform.rotation
            quaternion = (
                lefty.transform.rotation.x,
                lefty.transform.rotation.y,
                lefty.transform.rotation.z,
                lefty.transform.rotation.w
            )
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            self.get_logger().info(f'Roll: {math.degrees(roll)}, Pitch: {math.degrees(pitch)}, Yaw: {math.degrees(yaw)}')
            self.get_logger().info(f'Front axle center coordinates (absolute): x={_x}, y={_y}, z={_z}')
            self.get_logger().info(f'No. of waypoints: {len(self.waypoints)}')
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
