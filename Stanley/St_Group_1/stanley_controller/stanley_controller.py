#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import csv
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformException, Buffer, TransformListener

class StanleyController(Node):
    """
    Implement Stanley Control to follow a path
    """
    def __init__(self):
        super().__init__('stanley_controller_node')

        # Create a timer to run the controller function
        self.create_timer(0.001, self.control)
        self.last_dist_sq = None
        self.goal=0
        # Create a publisher for the drive command
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )
        
        self.flh = 'ego_racecar/front_left_hinge'
        self.frh = 'ego_racecar/front_right_hinge'
        self.fmap = 'map'
        self.base_link = 'ego_racecar/base_link'

        self.wayfile ='/home/rohan/sim_ws/src/stanley_controller/resource/waypoints.csv'
        self.waypoints=[]
        # Define parameters for Stanley Control
        self.k = 1.0  # Steering gain
        self.k_s = 1.0  # Softening constant for velocity
        self.max_steering_angle = np.deg2rad(30)  # Max steering angle in radians
        self.last_target_x, self.last_target_y = None, None
        self.velocity = 1.0  # Default velocity, adjust as needed
        self.current_pose = [0,0,0,0] #x,y,z,yaw
        
        # Initialize Transform Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize state variables
        self.map_frame_ready = False

        # Create a timer to check if the map frame is ready
        self.timer=self.create_timer(0.2, self.check_map_frame)

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

    def control(self):
        if not self.map_frame_ready:
            self.get_logger().warn('Waiting for map frame to be available.')
            return
        
        # Find the target point (simplified to a fixed point ahead)
        target_x, target_y = self.get_target_point()
        if self.last_target_x is None:
            self.last_target_x, self.last_target_y = target_x, target_y

        # Calculate cross-track error
        cross_track_error = self.calculate_cross_track_error()
        # Calculate heading error
        heading_error = self.calculate_heading_error()
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

    def euler_from_quaternion(x, y, z, w):
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

    def get_target_point(self):
        with open(file=self.wayfile,mode='r') as file:
            reader = csv.reader(file)
            self.waypoints = list(reader)
        self.target_x = float(self.waypoints[self.goal][0])
        self.target_y = float(self.waypoints[self.goal][1])
        dist_sq=(self.current_pose[0]-self.target_x)**2+((self.current_pose[1]-self.target_y)**2)
        if(self.last_dist_sq):
            if(dist_sq>self.last_dist_sq): #shift to next point moving away from current point
                self.goal+=1
                if(self.goal==len(self.waypoints)):
                    self.goal=0
                self.target_x = float(self.waypoints[self.goal][0])
                self.target_y = float(self.waypoints[self.goal][1])
        self.last_dist=dist_sq     

    def calculate_cross_track_error(self):
        return abs((self.last_target_y - self.target_y) * self.current_pose[0] - (self.last_target_x - self.target_x) * self.current_pose[1] + self.last_target_x * self.target_y - self.last_target_y * self.target_x) / np.sqrt((self.last_target_y - self.target_y) ** 2 + (self.last_target_x - self.target_x) ** 2)

    def calculate_heading_error(self):
        target_yaw = np.arctan2(self.target_y - self.current_pose[1], self.target_x - self.current_pose[0])
        return target_yaw - self.current_pose[3]

    def transform(self):
        try:
            transform_base = self.tf_buffer.lookup_transform(self.fmap, self.base_link, rclpy.time.Time())
            transform_left = self.tf_buffer.lookup_transform(self.base_link, self.flh, rclpy.time.Time())
            transform_right = self.tf_buffer.lookup_transform(self.base_link, self.frh, rclpy.time.Time())

            self.x_0 = (transform_left.transform.translation.x + transform_right.transform.translation.x)/2
            self.y_0 = (transform_left.transform.translation.y + transform_right.transform.translation.y)/2
            self.z_0 = (transform_left.transform.translation.z + transform_right.transform.translation.z)/2

            self.current_pose[0] = (transform_base.transform.translation.x + self.x_0)
            self.current_pose[1] = (transform_base.transform.translation.y + self.y_0)
            self.current_pose[2] = (transform_base.transform.translation.z + self.z_0)
            # Log the center of the front axle
            self.get_logger().info("Front Axle Center Coordinates: x: {x}, y: {y}, z: {z}")
        except TransformException as ex:
            self.get_logger().info(f'Transformation failed: {ex}')

         # Extract the rotation in quaternion form
        qx = transform_base.transform.rotation.x
        qy = transform_base.transform.rotation.y
        qz = transform_base.transform.rotation.z
        qw = transform_base.transform.rotation.w

        #convert it into euler angle
        _,_,self.current_pose[3]=self.euler_from_quaternion(qx, qy, qz, qw)


def main(args=None):
    rclpy.init(args=args)
    print("Stanley Controller Initialized")
    stanley_controller_node = StanleyController()
    rclpy.spin(stanley_controller_node)

    stanley_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

