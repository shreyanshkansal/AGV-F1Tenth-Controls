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
from nav_msgs.msg import Odometry
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
        self.ks = 1
        self.vel_max = 5
        self.min_vel_ratio = 0.1
        self.vel = 0.5
        self.angle_limit = np.pi/7
        self.flw = 'ego_racecar/front_left_wheel'
        self.frw = 'ego_racecar/front_right_wheel'
        self.blw = 'ego_racecar/back_left_wheel'
        self.brw = 'ego_racecar/back_right_wheel'
        self.bl = 'ego_racecar/base_link'
        self.ff = 'map'
        self.wayfile = '/sim_ws/src/f1tenth_stanley_controller/resource/waypoints.csv'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #Assuming that the waypoints are delivered to us in the right chronological sequence (probably assigned by some ID)
        self.waypoints = []
        self.vec:list
        self.getWaypoints()
        print(f'No. of waypoints: {len(self.waypoints)}')
        self.goal = 0
        qos = QoSProfile(depth=5)
        self.publisher = self.create_publisher(AckermannDriveStamped,'/drive',qos_profile=qos)
        self.vel_pub = self.create_publisher(Twist,'/cmd_vel',qos_profile=qos)
        self.goal_pub = self.create_publisher(PointStamped,'/local_goal_point',qos_profile=qos)
        self.timer = self.create_timer(timer_period_sec=0.1,callback=self.compute)

    def getWaypoints(self):
        with open(file=self.wayfile,mode='r') as file:
            reader = csv.reader(file)
            self.waypoints = list(reader)
            
    # def getTwist(self,msg:Odometry):
    #     x = float(msg.twist.twist.linear.x)
    #     y = float(msg.twist.twist.linear.y)
    #     self.vel_vector = [x,y,0.0]
    #     return
    
    def compute(self):
    
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
                t_blw = self.tf_buffer.lookup_transform(self.ff,self.blw,rclpy.time.Time(seconds=0))
                t_brw = self.tf_buffer.lookup_transform(self.ff,self.brw,rclpy.time.Time(seconds=0))
                break
            except TransformException as e:
                self.get_logger().warn(f'Could not transform: {e}')

        x0 = float(t_flw.transform.translation.x + t_frw.transform.translation.x)/2
        y0 = float(t_flw.transform.translation.y + t_frw.transform.translation.y)/2 
        z0 = float(t_flw.transform.translation.z + t_frw.transform.translation.z)/2
        xb0 = float(t_blw.transform.translation.x + t_brw.transform.translation.x)/2
        yb0 = float(t_blw.transform.translation.y + t_brw.transform.translation.y)/2 
        self.vec = [x0 - xb0,y0 - yb0,0.0]
        if self.goal < len(self.waypoints) - 1:
            while self.goal < len(self.waypoints)-1:
                x1 = float(self.waypoints[self.goal][0])
                y1 = float(self.waypoints[self.goal][1])
                x2 = float(self.waypoints[self.goal+1][0])
                y2 = float(self.waypoints[self.goal+1][1])
                dist1 = np.sqrt(np.square(x1-x0) + np.square(y1-y0))
                dist2 = np.sqrt(np.square(x2-x0) + np.square(y2-y0))
                if(dist1 < dist2): 
                    break
                else:
                    self.goal+=1

            # Goal waypoint visualisation
            p = PointStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.get_clock().now().to_msg()
            p.point.x = x1
            p.point.y = y1
            p.point.z = z0
            self.goal_pub.publish(p)
            print(f'\nGoal: {self.waypoints[self.goal]}')

            # Coordinate Geometry
            # Line equation: L: ax + by + c = 0
            a = y2 - y1
            b = x1 - x2
            c = y1*(x2-x1) - x1*(y2-y1)

            # Crosstrack error estimation
            e_t = abs(a*x0 + b*y0 + c)/np.sqrt(a*a + b*b)

            # Heading error estimation (Using vectors)
            # Dot product of the vector joining the two successive points and the vector representing the robot (not its velocity vector)
            dp = np.dot([x2-x1,y2-y1,0.0],self.vec)
            # Sign of the cross product determines the side of the path in which the robot is moving (right/clockwise or left/anticlockwise)
            vp_s = np.sign(np.cross([x2-x1,y2-y1,0.0],self.vec)[2])
            mag_line = np.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
            mag_vel = np.sqrt((self.vec[0])*(self.vec[0]) + (self.vec[1])*(self.vec[1]))
            cos = dp/(mag_line*mag_vel)
            psi_t:float = -vp_s*np.arccos(cos)
            

            print(f'Velocity = {self.vel}')
            print(f'Crosstrack Error = {e_t}, Heading Error = {psi_t*180/np.pi} deg')

            # Stanley Controller Algorithm
            delta_t = psi_t + np.sign(a*x0 + b*y0 + c)*np.arctan(self.k*(e_t/(self.ks + self.vel)))
            # Large heading error approximation
            if(abs(delta_t) >= self.angle_limit): 
                delta_t = np.sign(a*x0 + b*y0 + c)*self.angle_limit

            print(f'Output steering angle: {delta_t}\n')
            msg:AckermannDriveStamped = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.drive.steering_angle = float(delta_t)

            # This means that the change in steering angle in the sim happens almost instantly after the message is received (No delay)
            msg.drive.steering_angle_velocity = float(0.0) 

            # Reduction in velocity proportional to the heading error
            # Basically the bot slows down to (1/2)th of its initial speed at sharp, near 90 degree turns
            self.vel = self.vel_max*(1-(((1-self.min_vel_ratio)*abs(delta_t))/self.angle_limit))

            msg.drive.speed = self.vel
            msg.drive.acceleration = float(0.0)
            msg.drive.jerk = float(0.0)
            self.publisher.publish(msg)
            delta_t = 0
        else:
            SystemExit()
            return

def main(args=None):
    rclpy.init(args=args)
    print("Starting Stanley Node...")
    node = StanleyController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f'\nStopping the robot!')
        vel:Twist = Twist()
        node.vel_pub.publish(vel)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()