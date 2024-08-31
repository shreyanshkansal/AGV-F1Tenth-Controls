#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Pose, PoseStamped
from tf_transformations import euler_from_quaternion
from rclpy.publisher import Publisher



# TODO CHECK: include needed ROS msg-type headers and libraries

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers
        # Subscribing to the necessary topics
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Publishing to the /drive topic
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.max_steering_angle = 0.4
        self.l = 2.0
        
    def pose_callback(self, pose_msg):
        l = 2.0 
        self.max_steering_angle = 0.4  
        self.waypoints = np.array([
            [1.0, 1.0],
            [2.0, 0.5],
            [3.0, 0.0],
            [4.0, -1.0],
            [5.0, -2.0]
        ])
        # TODO: find the current waypoint to track using methods mentioned in lecture
        cx = pose_msg.pose.position.x
        cy = pose_msg.pose.position.y

        min_distance = float('inf')
        goal_point = None
        for waypoint in self.waypoints:
            distance = np.sqrt((waypoint[0] - cx) ** 2 + (waypoint[1] - cy) ** 2)
            if (distance > l) and (distance < min_distance) and (waypoint[0] - cx >= 0):
                min_distance = distance
                goal_point = waypoint
        
        # TODO: transform goal point to vehicle frame of reference
				
        # TODO: calculate curvature/steering angle

    def calculate_steering_angle(self, waypoint, lw, min_distance):
        angle_to_waypoint = math.atan2(waypoint[1], waypoint[0])
        steering_angle = math.atan2(2 * lw * math.sin(angle_to_waypoint), min_distance)
        return steering_angle
				
    def calculate_velocity(self, steering_angle, max_speed=10.0, min_speed=2.0):
        abs_steering_angle = abs(steering_angle)
        max_steering_angle = math.pi / 2  
        speed_factor = (max_steering_angle - abs_steering_angle) / max_steering_angle
        velocity = min_speed + speed_factor * (max_speed - min_speed)
        return velocity
	
    # TODO: publish drive message, don't forget to limit the steering angle.
    def publish_drive_message(self, steering_angle, velocity):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
									
def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
