
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
        super().__init__('stan')

        self.k= 0.1
        self.k_s = 2.0
        self.velocity = 2.5
        self.an_li = np.pi/7
        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.declare_parameter('left', 'ego_racecar/front_left_hinge')
        self.declare_parameter('right', 'ego_racecar/front_right_hinge')
        self.declare_parameter('base', 'ego_racecar/base_link')
        self.declare_parameter('map', 'map')

        self.wayfile = '~/sim_ws/src/my_stanley_controller/resource/waypoints.csv'
        self.waypoints = self.load_waypoints()
        self.waypoint_i = 0
        # Publisher
        self.drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        # Timer
        self.timer = self.create_timer(0.0001, self.stanley)

    def load_waypoints(self):
        """ Load waypoints from a CSV file efficiently """
        waypoints = pd.read_csv(self.wayfile, header=None).values
        return waypoints

    def get_nearest_waypoint(self, x, y):
        """ Find the nearest waypoint to the current position """
        min_dist = float('inf')
        nearest_index = 0
        for i in range(len(self.waypoints) - 1):
            dist = np.sqrt((x - float(self.waypoints[i][0])) ** 2 + (y - float(self.waypoints[i][1])) ** 2)
            if dist < min_dist:
                min_dist = dist
                nearest_index = i
        return nearest_index, float(self.waypoints[nearest_index][0]), float(self.waypoints[nearest_index][1])

    def stanley(self):
        left = self.get_parameter('left').get_parameter_value().string_value
        right = self.get_parameter('right').get_parameter_value().string_value
        map = self.get_parameter('map').get_parameter_value().string_value        
        base = self.get_parameter('base').get_parameter_value().string_value
        try:
            # Wait for the transform to be available
            self.tf_buffer.wait_for_transform_async(map, left, rclpy.time.Time())
            self.tf_buffer.wait_for_transform_async(map, right, rclpy.time.Time())
            self.tf_buffer.wait_for_transform_async(map, base, rclpy.time.Time())

            # Looking up transforms
            lefty = self.tf_buffer.lookup_transform(map, left, rclpy.time.Time())
            righty = self.tf_buffer.lookup_transform(map, right, rclpy.time.Time())
            basey = self.tf_buffer.lookup_transform(map, base, rclpy.time.Time())

            # Calculate the center of the front axle
            _x = (lefty.transform.translation.x + righty.transform.translation.x) / 2
            _y = (lefty.transform.translation.y + righty.transform.translation.y) / 2
            _z = (lefty.transform.translation.z + righty.transform.translation.z) / 2


            # If I need to broadcast the transform for the front axle center
            at = TransformStamped()
            at.header.stamp = self.get_clock().now().to_msg()
            at.header.frame_id = map
            at.child_frame_id = 'axle'

            at.transform.translation.x = _x
            at.transform.translation.y = _y
            at.transform.translation.z = _z

            at.transform.rotation = basey.transform.rotation
            quaternion = (
                basey.transform.rotation.x,
                basey.transform.rotation.y,
                basey.transform.rotation.z,
                basey.transform.rotation.w
            )
            roll, pitch, yaw = euler_from_quaternion(quaternion)

            x0 = _x
            y0 = _y

            idk = (5 * self.velocity)
            if idk<1:
                idk =1
            self.waypoint_i = self.waypoint_i % (len(self.waypoints)-1)
            if self.waypoint_i < len(self.waypoints):
                x1, y1 = self.waypoints[self.waypoint_i % (len(self.waypoints)-1)]
                x2, y2 = self.waypoints[(self.waypoint_i + idk) % (len(self.waypoints)-1)]

                dist1 = np.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
                dist2 = np.sqrt((x2 - x0) ** 2 + (y2 - y0) ** 2)

                if dist1 > dist2:
                    self.waypoint_i += 1
                    x1, y1 = self.waypoints[self.waypoint_i]
                    x2, y2 = self.waypoints[self.waypoint_i + idk]
            
                m = (y2 - y1) / (x2 - x1)
                a = y2 -y1
                b = x1 -x2
                c = (y1 - m * x1)
                cte = abs(m * x0 - y0 + c) / np.sqrt(1 + m * m)

        
            #self.get_logger().info(f'Roll: {math.degrees(roll)}, Pitch: {math.degrees(pitch)}, Yaw: {math.degrees(yaw)}')
            #self.get_logger().info(f'Front axle center coordinates (absolute): x={_x}, y={_y}, z={_z}')
            psi_t = np.arctan((m - np.tan(yaw)) / (1 + m * np.tan(yaw)))
            steer = psi_t + np.arctan(self.k * cte / (self.k_s + self.velocity))
            steer = np.sign(steer) * min(abs(steer), self.an_li)

                # Publish drive command
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.steering_angle = steer
            drive_msg.drive.speed = self.velocity
            self.drive.publish(drive_msg)
            #self.get_logger().info(f'{self.waypoint_i}')

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

