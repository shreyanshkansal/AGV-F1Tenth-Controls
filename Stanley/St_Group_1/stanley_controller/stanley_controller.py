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
        self.last_dist_sq = None
        self.goal=0
        # Create a publisher for the drive command
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )
        
        self.flh = "ego_racecar/front_left_hinge"
        self.frh = "ego_racecar/front_right_hinge"
        self.fmap = 'map'
        self.base_link = "ego_racecar/base_link"

        self.wayfile ='/home/rohan/sim_ws/src/stanley_controller/resource/waypoints.csv'
        self.waypoints=[]
        # Define parameters for Stanley Control
        self.k_head = 1   
        self.k_cross_track = 1# Steering gain
        self.k_cross_track_multiplier = 1
        self.k_s = 0.1  # Softening constant for velocity
        self.max_steering_angle = np.deg2rad(45)  # Max steering angle in radians
        self.last_target_x, self.last_target_y = None, None
        self.velocity = 1.2  # Default velocity, adjust as needed
        self.current_pose = [0,0,0,0] #x,y,z,yaw
        self.lookahead_time=0.3
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
            self.create_timer(0.001, self.control)
        except TransformException:
            self.get_logger().warn('Map frame is not available yet.')

    def control(self):
        if not self.map_frame_ready:
            self.get_logger().warn('Waiting for map frame to be available.')
            return
        
        # Find the target point (simplified to a fixed point ahead)
        self.get_target_point()
        self.transform()

        # Calculate cross-track error
        cross_track_error = self.calculate_cross_track_error()
        # Calculate heading error
        heading_error = self.calculate_heading_error()
        # Calculate steering angle using Stanley control formula
        steering_angle = self.k_head*heading_error + self.k_cross_track_multiplier*np.arctan2(self.k_cross_track * cross_track_error, self.k_s + self.velocity)
        # Limit the steering angle
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        # Publish the drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = self.fmap
        #drive_msg.drive.acceleration = float(0)  # Ensure acceleration is a float
        #drive  _msg.drive.jerk = float(0)  # Ensure jerk is a float
        drive_msg.drive.steering_angle = steering_angle
        #drive_msg.drive.steering_angle_velocity = float(0)  # Ensure steering_angle_velocity is a float
        drive_msg.drive.speed = self.velocity  # Use current velocity
        self.drive_publisher.publish(drive_msg)

        # Log the published message
        self.get_logger().info(f'Publishing steering angle: {steering_angle:.2f} and speed: {self.velocity:.2f}.')

        self.last_target_x, self.last_target_y = self.target_x, self.target_y

    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        # Convert quaternion to Euler angles
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z

    @staticmethod
    def distance_along_line(x1, y1, x2, y2, px, py):
        """
        Calculate the distance of point (px, py) along the line segment joining points (x1, y1) and (x2, y2).
        Positive distance means the point is closer to (x1, y1) than to (x2, y2) and not between them.
        Zero distance means the point is exactly at (x1, y1).
        Negative distance means the point is between (x1, y1) and (x2, y2) or closer to (x2, y2).
        
        :param x1: x-coordinate of the first point.
        :param y1: y-coordinate of the first point.
        :param x2: x-coordinate of the second point.
        :param y2: y-coordinate of the second point.
        :param px: x-coordinate of the point to measure the distance from.
        :param py: y-coordinate of the point to measure the distance from.
        :return: The distance of point (px, py) along the line.
        """
        if(x1,y1==x2,y2):
            return np.sqrt((x1-px)**2+(y1-py)**2)
        # Vector from (x1, y1) to (x2, y2)
        line_dx = x2 - x1
        line_dy = y2 - y1

        # Vector from (x1, y1) to (px, py)
        point_dx = px - x1
        point_dy = py - y1

        # Calculate the dot product and squared length of the line segment
        dot_product = line_dx * point_dx + line_dy * point_dy
        line_length_squared = line_dx * line_dx + line_dy * line_dy

        # Calculate the distance along the line
        if line_length_squared == 0:
            return 0.0

        projection = dot_product / line_length_squared

        # Check if the point is between the two points on the line segment
        if 0 <= projection <= 1:
            return -projection
        else:
            return projection


    def get_target_point(self):  
        with open(file=self.wayfile,mode='r') as file:
            reader = csv.reader(file)
            self.waypoints = list(reader)
        self.target_x = float(self.waypoints[self.goal][0])
        self.target_y = float(self.waypoints[self.goal][1])
        if self.last_target_x is None:
            self.last_target_x, self.last_target_y = self.target_x, self.target_y
        dist_sq=StanleyController.distance_along_line(self.target_x,self.target_y,self.last_target_x,self.last_target_y,self.current_pose[0],self.current_pose[1])
        while(dist_sq<self.velocity*self.lookahead_time):
            self.goal+=1
            if(self.goal==len(self.waypoints)):
                self.goal=0
            self.last_target_x=self.target_x
            self.last_target_y=self.target_y
            self.target_x = float(self.waypoints[self.goal][0])
            self.target_y = float(self.waypoints[self.goal][1])
            dist_sq=StanleyController.distance_along_line(self.target_x,self.target_y,self.last_target_x,self.last_target_y,self.current_pose[0],self.current_pose[1])
            self.last_dist_sq=dist_sq
        if(self.last_dist_sq):
            if(dist_sq>self.last_dist_sq): #shift to next point moving away from current point
                self.goal+=1
                if(self.goal==len(self.waypoints)):
                    self.goal=0
                self.last_target_x=self.target_x
                self.last_target_y=self.target_y
                self.target_x = float(self.waypoints[self.goal][0])
                self.target_y = float(self.waypoints[self.goal][1])
                dist_sq=StanleyController.distance_along_line(self.target_x,self.target_y,self.last_target_x,self.last_target_y,self.current_pose[0],self.current_pose[1])
        self.last_dist_sq=dist_sq     

    def calculate_cross_track_error(self):
        denom=np.sqrt((self.last_target_y - self.target_y) ** 2 + (self.last_target_x - self.target_x) ** 2)
        if denom!=0:
            return abs((self.last_target_y - self.target_y) * self.current_pose[0] - (self.last_target_x - self.target_x) * self.current_pose[1] + self.last_target_x * self.target_y - self.last_target_y * self.target_x) /denom
        else:
            return 0

    def calculate_heading_error(self):
        target_yaw = np.arctan2(self.target_y - self.current_pose[1], self.target_x - self.current_pose[0])
        return target_yaw - self.current_pose[3]

    def transform(self):
        try:
            transform_base = self.tf_buffer.lookup_transform(self.fmap, self.base_link, rclpy.time.Time())
            transform_left = self.tf_buffer.lookup_transform(self.base_link, self.flh, rclpy.time.Time())
            transform_right = self.tf_buffer.lookup_transform(self.base_link, self.frh, rclpy.time.Time())

            # Check if any transformation has NaN values
            for t in [transform_base, transform_left, transform_right]:
                if any(np.isnan([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
                                t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])):
                    raise ValueError("Transform contains NaN values")

            self.x_0 = (transform_left.transform.translation.x + transform_right.transform.translation.x) / 2
            self.y_0 = (transform_left.transform.translation.y + transform_right.transform.translation.y) / 2
            self.z_0 = (transform_left.transform.translation.z + transform_right.transform.translation.z) / 2

            self.current_pose[0] = (transform_base.transform.translation.x + self.x_0)
            self.current_pose[1] = (transform_base.transform.translation.y + self.y_0)
            self.current_pose[2] = (transform_base.transform.translation.z + self.z_0)

            # Log the center of the front axle
            #self.get_logger().info(f"Front Axle Center Coordinates: x: {self.x_0}, y: {self.y_0}, z: {self.z_0}")

            # Extract the rotation in quaternion form
            qx = transform_base.transform.rotation.x
            qy = transform_base.transform.rotation.y
            qz = transform_base.transform.rotation.z
            qw = transform_base.transform.rotation.w

            # Convert it into Euler angle
            _, _, self.current_pose[3] = StanleyController.euler_from_quaternion(qx, qy, qz, qw)

        except (TransformException, ValueError) as ex:
            self.get_logger().warn(f'Transformation failed: {ex}')

def main(args=None):
    rclpy.init(args=args)
    print("Stanley Controller Initialized")
    stanley_controller_node = StanleyController()
    rclpy.spin(stanley_controller_node)

    stanley_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

