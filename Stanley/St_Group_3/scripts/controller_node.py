import rclpy
from rclpy.node import Node
import csv
import numpy as np
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from transforms3d import euler


# TODO CHECK: include needed ROS msg type headers and libraries

class StanleyController(Node):
    # flw = 'ego_racecar/front_left_wheel'
    # frw = 'ego_racecar/front_right_wheel'
    # ff = 'map'
    def __init__(self):
        super().__init__('stanley_controller')
        # TODO: create ROS subscribers and publishers
        self.k = 2
        self.ks = 0.5
        self.vel = 0.5
        self.angle_limit = np.pi/6
        self.flw = 'ego_racecar/front_left_wheel'
        self.frw = 'ego_racecar/front_right_wheel'
        self.bl = 'ego_racecar/base_link'
        self.ff = 'map'
        self.wayfile = '/sim_ws/src/f1tenth_stanley_controller/resource/waypoints.csv'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_Buffer, self)
        #Assuming that the waypoints are delivered to us in the right chronological sequence (probably assigned by some ID)
        self.waypoints = []
        self.goal = 0

        self.publisher = self.create_publisher(AckermannDriveStamped,'/drive',10)
        self.vel_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.timer = self.create_timer(0.010,self.compute)

    def getWaypoints(self):
        with open(file=self.wayfile,mode='r') as file:
            reader = csv.reader()
            self.waypoints = list(reader)
    def compute(self):
        pass
        # TODO: find the current waypoint to track using methods mentioned in lecture
        try:
            #Syntax: First param: fixed frame/reference frame, second param: target frame
            t_flw = self.tf_buffer.lookup_transform(self.ff,self.flw,rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error(f'Could not transform: {e}')
        try:
            #Syntax: First param: fixed frame/reference frame, second param: target frame
            t_frw = self.tf_buffer.lookup_transform(self.ff,self.frw,rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error(f'Could not transform: {e}')
        try:
            #Syntax: First param: fixed frame/reference frame, second param: target frame
            t_heading = self.tf_buffer.lookup_transform(self.ff,self.bl,rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error(f'Could not transform: {e}')
        
        x0 = (t_flw.transform.translation.x + t_frw.tranform.translation.x)/2
        y0 = (t_flw.transform.translation.y + t_frw.tranform.translation.y)/2 
        if self.goal < len(self.waypoints) - 1:
            #z-coordinate is not needed
            z0 = (t_flw.transform.translation.z + t_frw.tranform.translation.z)/2
            quart:Quaternion = Quaternion()
            quart.x = t_heading.transform.quaternion.x
            quart.y = t_heading.transform.quaternion.y
            quart.z = t_heading.transform.quaternion.z
            quart.w = t_heading.transform.quaternion.w
            _,_,yaw = euler.quat2euler([quart.x,quart.y,quart.z,quart.w],'xyzw')
            while True:
                x1 = self.waypoints[self.goal][0]
                y1 = self.waypoints[self.goal][1]
                x2 = self.waypoints[self.goal+1][0]
                y2 = self.waypoints[self.goal+1][1]
                dist1 = np.sqrt(np.power(x1-x0,2)+np.power(y1-y0,2))
                dist2 = np.sqrt(np.power(x2-x0,2)+np.power(y2-y0,2))
                if dist1<dist2:
                    break
            m = (y2 - y1)/(x2 - x1)
            c = y1 - m*x1
            e_t = abs(m*x0 - y0 + c)/np.sqrt(1 + m*m)
            psi_t = np.arctan((m-np.tan(yaw))/1+m*np.tan(yaw))
            delta_t = psi_t + np.arctan(self.k*e_t/(self.ks + self.vel))
            delta_t = np.sign(delta_t)*max(abs(delta_t),self.angle_limit)
            msg:AckermannDriveStamped = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.drive.steering_angle = delta_t
            msg.drive.steering_angle_velocity = 0 #tries to steer at maximum possible speed (most likely instantaneous change)
            msg.drive.speed = self.vel
            msg.drive.acceleration = 0
            msg.drive.jerk = 0
            twist:Twist = Twist()
            twist.linear.x = self.vel
            twist.linear.y = 0
            twist.linear.y = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.publisher.publish(msg)
            self.vel_pub.publish()
        else:
            return
        # TODO: transform goal point to vehicle frame of reference
        
        # TODO: calculate curvature/steering angle

        # TODO: publish drive message, don't forget to limit the steering angle.

def main(args=None):
    rclpy.init(args=args)
    print("Starting Stanley Node...")
    node = StanleyController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()