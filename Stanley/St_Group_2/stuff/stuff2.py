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
import math

class Stanley(Node):
    
    def __init__(self):
        super().__init__('stan')
        
        #self.path = self.load_path()
        self.waypoints = [( 0, 0), ( 0.5, 0), ( 1, 0), ( 1.5, 0), ( 2, 0), ( 2.5, 0), ( 2.92073, 0), ( 3.17073, 0), ( 3.42073, 0), ( 3.67073, 0), ( 3.92073, 0), ( 4.17073, 0), ( 4.42073, 0), ( 4.67073, 0), ( 4.92073, 0), ( 5.17073, 0), ( 5.42073, 0), ( 5.67073, 0), ( 5.92073, 0), ( 6.17073, 0), ( 6.42073, 0), ( 6.67073, 0), ( 6.92073, 0), ( 7.17073, 0), ( 7.42073, 0), ( 7.67073, 0), ( 7.92073, 0), ( 8.17073, 0), ( 8.42021, 0.0108947), ( 8.6611, 0.0755442), ( 8.88001, 0.195064), ( 9.06464, 0.362742), ( 9.20464, 0.56916), ( 9.29759, 0.800978), ( 9.38173, 1.03639), ( 9.46587, 1.27181), ( 9.54992, 1.50726), ( 9.60405, 1.75073), ( 9.60931, 2.00055), ( 9.60916, 2.25055), ( 9.609, 2.50055), ( 9.60885, 2.75055), ( 9.6087, 3.00055), ( 9.60854, 3.25055), ( 9.60839, 3.50055)]
        self.subscription = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)   
        self.subscription
        self.drive_publisher1 = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.drive_publisher2 = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.declare_parameter('target_frame', 'ego_racecar/base_link')
        self.declare_parameter('source_frame_left', 'ego_racecar/front_left_hinge')
        self.declare_parameter('source_frame_right', 'ego_racecar/front_right_hinge')

        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.current_odom = None 
        self.k = 0.5  # Control gain
        self.wheelbase = 0.33  # Wheelbase of the F1TENTH car
        self.desired_speed = 1.0 
        
    def compute_cte_and_path_yaw(self,x,y):
        
        closest_dist = float('inf')
        closest_point = None
        index = 0
        point1 = np.array([x,y])
        point2 = np.array([0,0])
        
        for (px,py) in self.waypoints:
            
            index = index + 1
            ki = 1
            point2[0] ,point2[1] = px ,py
            dist =np.linalg.norm(point2  - point1)
            
            if dist < closest_dist:
                closest_dist = dist
                closest_point = (px,py)
                opt_index = index
                
        cte = closest_dist
        ydist = self.waypoints[opt_index + ki][1] - self.waypoints[opt_index][1]
        xdist = self.waypoints[opt_index + ki][0] - self.waypoints[opt_index][0]
        path_yaw = math.atan(ydist/xdist)
        return cte,path_yaw
        
        
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
            #self.get_logger().info(f'Front axle center coordinates (absolute): x={center_x_absolute}, y={center_y_absolute}, z={center_z_absolute}')
            
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
            return
        
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        yaw = math.atan2(2.0*(y*z + w*x), w*w - x*x - y*y + z*z);        
        cte, path_yaw = self.compute_cte_and_path_yaw(center_x_absolute,center_y_absolute)
        
        steer_angle = path_yaw - yaw + np.arctan2(self.k * cte, self.desired_speed)
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = max(min(steer_angle, np.pi / 6), -np.pi / 6)  # Limit the steering angle
        self.drive_publisher1.publish(drive_msg)
        
        speed_msg = Twist()
        speed_msg.linear.x = self.desired_speed
        self.drive_publisher2.publish(speed_msg)
        
def main(args=None):
    rclpy.init(args=args)
    print("Stanley Initialized")
    node = Stanley()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()