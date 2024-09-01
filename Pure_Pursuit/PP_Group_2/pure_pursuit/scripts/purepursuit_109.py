#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Pose, Point
from tf_transformations import euler_from_quaternion


class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        # Parameters
        self.lookahead_distance = 2.0  # Lookahead distance in meters
        self.max_steering_angle = 0.4  # Max steering angle in radians
        self.speed = 1.0  # Speed of the car in meters per second

        # Subscribing and Publishing to the necessary topics
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Load waypoints from JSON file
        self.waypoints = self.load_waypoints_from_json('/home/spk/sim_ws/src/f1tenth_lab6_template/pure_pursuit/scripts/waypoints.json')

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

        min_distance = float('inf')
        target_point = None

        for waypoint in self.waypoints:
            distance = np.sqrt((waypoint.position.x - self.current_pose.position.x) ** 2 +
                               (waypoint.position.y - self.current_pose.position.y) ** 2)
            if (distance > self.lookahead_distance) and (distance < min_distance):
                min_distance = distance
                target_point = waypoint

        if target_point is None:
            self.get_logger().info('No suitable waypoint found.')
            return

        # Transform goal point to vehicle frame of reference
        transformed_point = self.transform_to_vehicle_frame(target_point)

        # Calculate steering angle
        steering_angle = self.calculate_steering_angle(transformed_point)
        self.get_logger().info(f'Steering angle: {steering_angle:.2f}')

        # Publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        drive_msg.drive.speed = self.speed
        self.drive_pub.publish(drive_msg)

    def transform_to_vehicle_frame(self, target_point):
        dx = target_point.position.x - self.current_pose.position.x
        dy = target_point.position.y - self.current_pose.position.y

        yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)

        x_vehicle = dx * np.cos(-yaw) - dy * np.sin(-yaw)
        y_vehicle = dx * np.sin(-yaw) + dy * np.cos(-yaw)
        self.get_logger().info(f'Transformed point: x={x_vehicle:.2f}, y={y_vehicle:.2f}')

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
