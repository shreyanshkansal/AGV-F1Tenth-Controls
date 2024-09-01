import rclpy
from rclpy.node import Node
import csv
import numpy as np
from decimal import Decimal
from rclpy.qos import QoSProfile
import rclpy.time
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
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
        self.k = 3
        self.ks = 2.5
        self.vel_max = 0.5
        self.vel = 0.5
        self.angle_limit = np.pi/4
        self.flw = 'ego_racecar/front_left_wheel'
        self.frw = 'ego_racecar/front_right_wheel'
        self.bl = 'ego_racecar/base_link'
        self.ff = 'map'
        self.wayfile = '/sim_ws/src/f1tenth_stanley_controller/resource/waypoints.csv'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #Assuming that the waypoints are delivered to us in the right chronological sequence (probably assigned by some ID)
        self.waypoints = []
        self.getWaypoints()
        print(f'No. of waypoints: {len(self.waypoints)}')
        self.goal = 0
        qos = QoSProfile(depth=3)
        self.publisher = self.create_publisher(AckermannDriveStamped,'/drive',qos_profile=qos)
        self.vel_pub = self.create_publisher(Twist,'/cmd_vel',qos_profile=qos)
        self.goal_pub = self.create_publisher(PointStamped,'/local_goal_point',qos_profile=qos)
        # Higher the rate of execution, lower the error caused by delayed messages
        self.timer = self.create_timer(timer_period_sec=0.05,callback=self.compute)

    def getWaypoints(self):
        with open(file=self.wayfile,mode='r') as file:
            reader = csv.reader(file)
            self.waypoints = list(reader)
    
    def compute(self):
        try:
            t_flw = TransformStamped()
            t_frw = TransformStamped()
            t_heading = TransformStamped()
            # TODO: find the current waypoint to track using methods mentioned in lecture
            while rclpy.ok():
                try:
                    #Syntax: First param: fixed frame/reference frame, second param: target frame
                    # self.tf_buffer.wait_for_transform_async(self.ff,self.flw,rclpy.time.Time(seconds=0))
                    t_flw = self.tf_buffer.lookup_transform(self.ff,self.flw,rclpy.time.Time(seconds=0))
                    #Syntax: First param: fixed frame/reference frame, second param: target frame
                    # self.tf_buffer.wait_for_transform_async(self.ff,self.frw,rclpy.time.Time(seconds=0))
                    t_frw = self.tf_buffer.lookup_transform(self.ff,self.frw,rclpy.time.Time(seconds=0))
                    #Syntax: First param: fixed frame/reference frame, second param: target frame
                    # self.tf_buffer.wait_for_transform_async(self.ff,self.bl,rclpy.time.Time(seconds=0))
                    t_heading = self.tf_buffer.lookup_transform(self.ff,self.bl,rclpy.time.Time(seconds=0))
                    break
                except TransformException as e:
                    self.get_logger().warn(f'Could not transform: {e}')
            
            x0 = float(t_flw.transform.translation.x + t_frw.transform.translation.x)/2
            y0 = float(t_flw.transform.translation.y + t_frw.transform.translation.y)/2 
            if self.goal < len(self.waypoints) - 1:
                #z-coordinate is not needed
                z0 = (t_flw.transform.translation.z + t_frw.transform.translation.z)/2
                quart:Quaternion = Quaternion()
                quart.x = t_heading.transform.rotation.x
                quart.y = t_heading.transform.rotation.y
                quart.z = t_heading.transform.rotation.z
                quart.w = t_heading.transform.rotation.w
                _,_,yaw = euler.quat2euler([quart.w,quart.x,quart.y,quart.z])
                dist = float("inf")
                i:int
                for i in range(len(self.waypoints)):
                    x = float(self.waypoints[i][0])
                    y = float(self.waypoints[i][1])
                    dist_ = np.sqrt(np.power(x-x0,2)+np.power(y-y0,2))
                    if dist_<dist:
                        dist = dist_
                        self.goal = i
                self.goal+=1

                p = PointStamped()
                p.header.frame_id = 'map'
                p.header.stamp = self.get_clock().now().to_msg()
                x1 = float(self.waypoints[self.goal][0])
                y1 = float(self.waypoints[self.goal][1])
                p.point.x = x1
                p.point.y = y1
                p.point.z = z0
                self.goal_pub.publish(p)

                x2 = float(self.waypoints[self.goal+1][0])
                y2 = float(self.waypoints[self.goal+1][1])
                print(f'Goal: {self.waypoints[self.goal]}')
                m = (y2 - y1)/(x2 - x1)
                c = y1 - m*x1

                e_t = abs(m*x0 - y0 + c)/np.sqrt(1 + m*m)
                psi_t = np.arctan((m-np.tan(yaw))/(1+m*np.tan(yaw)))

                print(f'Velocity = {self.vel}')
                print(f'Crosstrack Error = {e_t}, Heading Error = {psi_t*180/np.pi} deg')

                # Stanley Controller Algorithm
                delta_t = psi_t + np.sign(m*x0 - y0 + c)*np.arctan(self.k*(e_t/(self.ks + self.vel)))
                if(abs(delta_t) >= self.angle_limit): 
                    delta_t = np.sign(m*x0 - y0 + c)*self.angle_limit
                print(f'Output steering angle: {delta_t}')
                msg:AckermannDriveStamped = AckermannDriveStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"
                msg.drive.steering_angle = delta_t

                # This means that the change in steering angle in the sim happens almost instantly after the message is received (No delay)
                msg.drive.steering_angle_velocity = float(0.0) 

                # Reduction in velocity proportional to the heading error
                # Basically the bot slows down to (1/2)th of its initial speed at sharp, near 90 degree turns
                self.vel = self.vel_max*(1-((0.5*abs(delta_t))/self.angle_limit))

                msg.drive.speed = self.vel
                msg.drive.acceleration = float(0.0)
                msg.drive.jerk = float(0.0)
                self.publisher.publish(msg)
                delta_t = 0
            else:
                return
        except KeyboardInterrupt:
            msg:AckermannDriveStamped = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.drive.steering_angle = 0
            msg.drive.steering_angle_velocity = float(0.0)
            self.vel = 0
            msg.drive.speed = self.vel
            msg.drive.acceleration = float(0.0)
            msg.drive.jerk = float(0.0)
            self.publisher.publish(msg)
            rclpy.shutdown()
        # TODO: transform goal point to vehicle frame of reference
        
        # TODO: calculate curvature/steering angle

        # TODO: publish drive message, don't forget to limit the steering angle.

def main(args=None):
    rclpy.init(args=args)
    print("Starting Stanley Node...")
    node = StanleyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        msg:AckermannDriveStamped = AckermannDriveStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.drive.steering_angle = 0
        msg.drive.steering_angle_velocity = float(0.0)
        vel = 0
        msg.drive.speed = vel
        msg.drive.acceleration = float(0.0)
        msg.drive.jerk = float(0.0)
        node.publisher.publish(msg)
        rclpy.shutdown()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()