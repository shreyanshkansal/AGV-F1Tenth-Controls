#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import json
import time
from datetime import datetime

class PoseSaver(Node):
    def __init__(self):
        super().__init__('pose_saver_node')
        
        # Parameters
        self.save_interval = 0.1  # Save poses every second
        self.last_save_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.pose_data = []

        # ROS Subscriber
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create directory and file if not exists
        self.file_path = '/home/aryan/sim_ws/src/f1tenth_lab6_template/pure_pursuit/scripts/marker_aruco.json'  # Update to your desired path
        self.create_file_if_not_exists()

    def listener_callback(self, msg):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.last_save_time >= self.save_interval:
            pose = {
                'timestamp': datetime.now().isoformat(),
                'pose': {
                    'position': {
                        'x': msg.pose.pose.position.x,
                        'y': msg.pose.pose.position.y,
                        'z': msg.pose.pose.position.z
                    },
                    
                }
            }
            self.pose_data.append(pose)
            self.save_to_json()
            self.last_save_time = current_time

    def save_to_json(self):
        with open(self.file_path, 'w') as file:
            json.dump(self.pose_data, file, indent=4)

    def create_file_if_not_exists(self):
        try:
            with open(self.file_path, 'r') as file:
                pass  # File exists, nothing to do
        except FileNotFoundError:
            with open(self.file_path, 'w') as file:
                json.dump([], file)  # Create an empty list in JSON file

def main(args=None):
    rclpy.init(args=args)
    node = PoseSaver()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
