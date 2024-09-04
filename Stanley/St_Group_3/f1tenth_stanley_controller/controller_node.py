import rclpy
import csv
import time
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
from ackermann_msgs.msg import AckermannDriveStamped

# TODO CHECK: include needed ROS msg type headers and libraries

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        # TODO: create ROS subscribers and publishers
        self.declare_parameter('k',5.0)
        self.declare_parameter('ks',10.0)
        self.declare_parameter('vel_max',5.0)
        self.declare_parameter('min_vel_ratio',0.25)
        self.angle_limit = np.pi/7
        self.flw = 'ego_racecar/front_left_wheel'
        self.frw = 'ego_racecar/front_right_wheel'
        self.blw = 'ego_racecar/back_left_wheel'
        self.brw = 'ego_racecar/back_right_wheel'
        self.bl = 'ego_racecar/base_link'
        self.ff = 'map'
        self.k = float(self.get_parameter('k').get_parameter_value().double_value)
        self.ks = float(self.get_parameter('ks').get_parameter_value().double_value)
        self.vel_max = float(self.get_parameter('vel_max').get_parameter_value().double_value)
        self.vel = self.vel_max
        self.min_vel_ratio = float(self.get_parameter('min_vel_ratio').get_parameter_value().double_value)
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
            
    def getNearestWaypoint(self,x,y):
        i = 0
        min_dist = float("inf")
        for j in range(len(self.waypoints)-1):
            dist_ = np.sqrt((x-float(self.waypoints[j][0]))*(x-float(self.waypoints[j][0])) + (y-float(self.waypoints[j][1]))*(y-float(self.waypoints[j][1])))
            if dist_ < min_dist:
                min_dist = dist_
                i = j
        return i,float(self.waypoints[i][0]),float(self.waypoints[i][1])
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
            self.goal,x1,y1 = self.getNearestWaypoint(x=x0,y=y0)
            num = int(np.power(1.3875,self.vel/(self.vel_max*self.min_vel_ratio)))
            num += self.goal
            num = np.minimum(num,len(self.waypoints)-1)
            x2 = float(self.waypoints[num][0])
            y2 = float(self.waypoints[num][1])
            # Goal waypoint visualisation
            p = PointStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.get_clock().now().to_msg()
            p.point.x = x2
            p.point.y = y2
            p.point.z = z0
            self.goal_pub.publish(p)

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
            
            # Stanley Controller Algorithm
            delta_t = psi_t + np.sign(a*x0 + b*y0 + c)*np.arctan(self.k*(e_t/(self.ks + self.vel)))
            if(abs(delta_t) >= self.angle_limit): 
                delta_t = np.sign(a*x0 + b*y0 + c)*self.angle_limit

            # Logs
            print(f'\nParams: k = {self.k}, ks = {self.ks}, vel(max) = {self.vel_max}, vel(min) = {self.vel_max*self.min_vel_ratio}')
            print(f'Goal: {self.waypoints[self.goal]}')
            print(f'Velocity = {self.vel}')
            print(f'Crosstrack Error = {e_t}, Heading Error = {psi_t*180/np.pi} deg')
            print(f'Output steering angle: {delta_t*180/np.pi}\n')

            # Publishing the steering angle and updated velocity
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
        time.sleep(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()