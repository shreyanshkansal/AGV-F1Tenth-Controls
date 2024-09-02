#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformException, Buffer, TransformListener

class StanleyController(Node):
    """
    Implement Stanley Control to follow a path
    """
    def __init__(self):
        super().__init__('stanley_controller_node')

        # Create a subscriber for the vehicle pose
        self.pose_subscriber = self.create_subscription(
            Odometry,  # Change to Odometry type
            '/ego_racecar/odom',
            self.pose_callback,
            10
        )
        
        # Create a publisher for the drive command
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )
        
        self.flw = 'ego_racecar/front_left_wheel'
        self.frw = 'ego_racecar/front_right_wheel'
        self.fmap = 'map'
        
        # Define parameters for Stanley Control
        self.k = 1.0  # Steering gain
        self.k_s = 1.0  # Softening constant for velocity
        self.max_steering_angle = np.deg2rad(30)  # Max steering angle in radians
        self.last_target_x, self.last_target_y = None, None
        self.current_pose = None
        self.velocity = 1.0  # Default velocity, adjust as needed

        # Initialize Transform Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize state variables
        self.map_frame_ready = False

        # Create a timer to check if the map frame is ready
        self.timer = self.create_timer(1.0, self.check_map_frame)

        # Log initialization
        self.get_logger().info('Stanley Controller Node has been initialized.')

    def check_map_frame(self):
        try:
            # Attempt to get the transform from map to map (just to check if map frame is available)
            self.tf_buffer.lookup_transform(self.fmap, self.fmap, rclpy.time.Time())
            self.map_frame_ready = True
            self.get_logger().info('Map frame is available.')
            # Stop the timer once the frame is available
            self.timer.cancel()
        except TransformException:
            self.get_logger().warn('Map frame is not available yet.')

    def pose_callback(self, pose_msg):
        if not self.map_frame_ready:
            self.get_logger().warn('Waiting for map frame to be available.')
            return

        try:
            t_flw = self.tf_buffer.lookup_transform(self.fmap, self.flw, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error(f'Could not transform front left wheel: {e}')
            return
        
        try:
            t_frw = self.tf_buffer.lookup_transform(self.fmap, self.frw, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error(f'Could not transform front right wheel: {e}')
            return
        
        # Front middle of axle is in the middle of the front right and left wheel
        x0 = (t_flw.transform.translation.x + t_frw.transform.translation.x) / 2
        y0 = (t_flw.transform.translation.y + t_frw.transform.translation.y) / 2
        # z-coordinate is not needed
        z0 = (t_flw.transform.translation.z + t_frw.transform.translation.z) / 2
        
        if self.current_pose is None:
            self.current_pose = pose_msg

        # Get the vehicle's current position and orientation
        x = x0  # pose_msg.pose.position.x
        y = y0  # pose_msg.pose.position.y
        yaw = self.get_yaw_from_pose(pose_msg)

        # Find the target point (simplified to a fixed point ahead)
        target_x, target_y = self.get_target_point(x, y)
        if self.last_target_x is None:
            self.last_target_x, self.last_target_y = target_x, target_y

        # Calculate cross-track error
        cross_track_error = self.calculate_cross_track_error(x, y, target_x, target_y)

        # Calculate heading error
        heading_error = self.calculate_heading_error(x, y, target_x, target_y, yaw)

        # Calculate steering angle using Stanley control formula
        steering_angle = heading_error + np.arctan2(self.k * cross_track_error, self.k_s + self.velocity)

        # Limit the steering angle
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        # Publish the drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "map"
        #drive_msg.drive.acceleration = float(0)  # Ensure acceleration is a float
        #drive_msg.drive.jerk = float(0)  # Ensure jerk is a float
        drive_msg.drive.steering_angle = steering_angle
        #drive_msg.drive.steering_angle_velocity = float(0)  # Ensure steering_angle_velocity is a float
        drive_msg.drive.speed = self.velocity  # Use current velocity
        self.drive_publisher.publish(drive_msg)

        # Log the published message
        self.get_logger().info(f'Publishing steering angle: {steering_angle:.2f} and speed: {self.velocity:.2f}.')

        self.last_target_x, self.last_target_y = target_x, target_y

    def get_yaw_from_pose(self, pose_msg):
        # Extract yaw (heading) from quaternion
        pose = pose_msg.pose.pose
        orientation = pose.orientation
        _, _, yaw = self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        return yaw

    def euler_from_quaternion(self, x, y, z, w):
        # Convert quaternion to Euler angles
        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z

    def get_target_point(self, x, y):
        # Simplified: In a real implementation, use path data to determine the target point
        # For now, we'll assume a fixed target point ahead
        target_x = x + 10.0  # Simple fixed point for demonstration
        target_y = y + 2 * np.random.rand()  
        return target_x, target_y

    def calculate_cross_track_error(self, x, y, target_x, target_y):
        # Calculate the cross-track error
        return abs((self.last_target_y - target_y) * x - (self.last_target_x - target_x) * y + self.last_target_x * target_y - self.last_target_y * target_x) / np.sqrt((self.last_target_y - target_y) ** 2 + (self.last_target_x - target_x) ** 2)

    def calculate_heading_error(self, x, y, target_x, target_y, yaw):
        # Calculate the heading error
        target_yaw = np.arctan2(target_y - y, target_x - x)
        return target_yaw - yaw

def main(args=None):
    rclpy.init(args=args)
    print("Stanley Controller Initialized")
    stanley_controller_node = StanleyController()
    rclpy.spin(stanley_controller_node)

    stanley_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()