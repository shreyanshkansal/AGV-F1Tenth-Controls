#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import numpy as np
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Pose, Point, Quaternion

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Parameters
        self.lookahead_distance = 2.0  # Lookahead distance
        self.max_steering_angle = 0.4189  # Max steering angle in radians (~24 degrees)
        self.speed = 1.0  # Speed of the car

        # Load waypoints from JSON file
        self.waypoints = self.load_waypoints_from_json('/home/aryan/sim_ws/src/f1tenth_lab6_template/pure_pursuit/scripts/marker_aruco.json')

        # ROS Publishers and Subscribers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        self.current_pose = None

    def load_waypoints_from_json(self, file_path):
        waypoints = []
        try:
            with open(file_path, 'r') as file:
                data = json.load(file)
                for entry in data:
                    pose = Pose(
                        position=Point(
                            x=entry['pose']['position']['x'],
                            y=entry['pose']['position']['y'],
                            z=entry['pose']['position']['z']
                        )
                    )
                    waypoints.append(pose)
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints from JSON: {e}')
        return waypoints

    def odom_callback(self, odom_msg):
        self.current_pose = odom_msg.pose.pose
        self.pose_callback()

    def pose_callback(self):
        if len(self.waypoints) == 0 or self.current_pose is None:
            return

        # Find the nearest point ahead of the car based on lookahead distance
        nearest_point, target_point = self.find_target_point()

        if target_point is None:
            # No valid target point found
            return

        # Transform target point to vehicle frame of reference
        transformed_point = self.transform_to_vehicle_frame(target_point)

        # Calculate steering angle
        steering_angle = self.calculate_steering_angle(transformed_point)

        # Create and publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        drive_msg.drive.speed = self.speed
        self.drive_pub.publish(drive_msg)

    def find_target_point(self):
        nearest_point = None
        target_point = None
        min_distance = float('inf')

        for point in self.waypoints:
            distance = np.sqrt((point.position.x - self.current_pose.position.x) ** 2 +
                               (point.position.y - self.current_pose.position.y) ** 2)
            if distance < min_distance and distance > self.lookahead_distance:
                min_distance = distance
                target_point = point
                nearest_point = point

        return nearest_point, target_point

    def transform_to_vehicle_frame(self, target_point):
        dx = target_point.position.x - self.current_pose.position.x
        dy = target_point.position.y - self.current_pose.position.y

        yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)

        x_vehicle = dx * np.cos(-yaw) - dy * np.sin(-yaw)
        y_vehicle = dx * np.sin(-yaw) + dy * np.cos(-yaw)

        return x_vehicle, y_vehicle

    def calculate_steering_angle(self, transformed_point):
        x_vehicle, y_vehicle = transformed_point
        curvature = 2 * y_vehicle / (self.lookahead_distance ** 2)
        steering_angle = np.arctan(curvature)
        return steering_angle

    def get_yaw_from_quaternion(self, quaternion):
        """
        Converts a quaternion into yaw (rotation around Z-axis)
        """
        qx, qy, qz, qw = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return np.arctan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
