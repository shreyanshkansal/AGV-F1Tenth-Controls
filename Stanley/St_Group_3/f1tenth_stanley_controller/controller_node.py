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

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        # Hyperparamters which can be set dynamically (in cmd line) at the time of execution
        self.declare_parameter('k',5.0)
        self.declare_parameter('ks',10.0)
        self.declare_parameter('vel_max',5.0)
        self.declare_parameter('min_vel_ratio',0.25)

        # Constants
        self.angle_limit = np.pi/6
        self.goal = 0
        self.avg_vel = 0
        self.count = 0
        self.flw = 'ego_racecar/front_left_wheel'
        self.frw = 'ego_racecar/front_right_wheel'
        self.blw = 'ego_racecar/back_left_wheel'
        self.brw = 'ego_racecar/back_right_wheel'
        self.bl = 'ego_racecar/base_link'
        self.ff = 'map'
        self.wayfile = '/sim_ws/src/f1tenth_stanley_controller/resource/Spielberg_Waypoints(3).csv'

        # Copying the entered parameter values to the variables
        self.k = float(self.get_parameter('k').get_parameter_value().double_value)
        self.ks = float(self.get_parameter('ks').get_parameter_value().double_value)
        self.vel_max = float(self.get_parameter('vel_max').get_parameter_value().double_value)
        self.vel = self.vel_max
        self.min_vel_ratio = float(self.get_parameter('min_vel_ratio').get_parameter_value().double_value)
        
        # Transform Listener initialization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Declaration
        self.waypoints = []
        self.vec:list

        # Obtaining waypoint oordinates in a list from the CSV file
        self.getWaypoints()
        print(f'No. of waypoints: {len(self.waypoints)}') # Test statement

        # Initialization of publishers and looped callback
        qos = QoSProfile(depth=5)
        self.publisher = self.create_publisher(AckermannDriveStamped,'/drive',qos_profile=qos)
        self.vel_pub = self.create_publisher(Twist,'/cmd_vel',qos_profile=qos)
        self.goal_pub = self.create_publisher(PointStamped,'/local_goal_point',qos_profile=qos)
        time.sleep(1) # waits for the frames to be intialized and ready for tranformations
        self.timer = self.create_timer(timer_period_sec=0.01,callback=self.compute)

    def getWaypoints(self):
        with open(file=self.wayfile,mode='r') as file:
            reader = csv.reader(file)
            self.waypoints = list(reader) # directly converts the CSV file object into a two dimensional list (list of rows, each row is a list)
            
    def getNearestWaypoint(self,x,y):
        i = 0
        min_dist = float("inf")
        for j in range(len(self.waypoints)-1):
            dist_ = np.sqrt((x-float(self.waypoints[j][0]))*(x-float(self.waypoints[j][0])) + (y-float(self.waypoints[j][1]))*(y-float(self.waypoints[j][1])))
            if dist_ < min_dist:
                min_dist = dist_
                i = j
        return i,float(self.waypoints[i][0]),float(self.waypoints[i][1]) # returns the index of the waypoint in the list, and its coordinates
    
    def compute(self):
        t_flw = TransformStamped()
        t_frw = TransformStamped()
        t_blw = TransformStamped()
        t_brw = TransformStamped()
        while rclpy.ok():

            try:
                # Syntax: First param: fixed frame/reference frame, second param: target frame
                t_flw = self.tf_buffer.lookup_transform(self.ff,self.flw,rclpy.time.Time(seconds=0))
                t_frw = self.tf_buffer.lookup_transform(self.ff,self.frw,rclpy.time.Time(seconds=0))
                t_blw = self.tf_buffer.lookup_transform(self.ff,self.blw,rclpy.time.Time(seconds=0))
                t_brw = self.tf_buffer.lookup_transform(self.ff,self.brw,rclpy.time.Time(seconds=0))
                break

            except TransformException as e:
                self.get_logger().warn(f'Could not transform: {e}')
        # Coordinates of the midpoint of the front axle
        x0 = float(t_flw.transform.translation.x + t_frw.transform.translation.x)/2
        y0 = float(t_flw.transform.translation.y + t_frw.transform.translation.y)/2 
        z0 = float(t_flw.transform.translation.z + t_frw.transform.translation.z)/2

        # Coordinates of the midpoint of the back axle (required only for obtaining the robot heading vector)
        xb0 = float(t_blw.transform.translation.x + t_brw.transform.translation.x)/2
        yb0 = float(t_blw.transform.translation.y + t_brw.transform.translation.y)/2 

        # Vector representing robot heading (independent of velocity magnitude)
        self.vec = [x0 - xb0,y0 - yb0,0.0]

        # Retreiving closest waypoint data
        self.goal,x1,y1 = self.getNearestWaypoint(x=x0,y=y0)
        num = int(np.power(1.05,self.vel/(self.vel_max*self.min_vel_ratio)))
        num += self.goal
        num = np.minimum(num,len(self.waypoints)-1)
        x2 = float(self.waypoints[num][0])
        y2 = float(self.waypoints[num][1])

        # Next waypoint visualisation
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
        mag_heading = np.sqrt((self.vec[0])*(self.vec[0]) + (self.vec[1])*(self.vec[1]))
        cos = dp/(mag_line*mag_heading)
        psi_t:float = -vp_s*np.arccos(cos)
        
        # Stanley Controller Algorithm
        delta_t = psi_t + np.sign(a*x0 + b*y0 + c)*np.arctan(self.k*(e_t/(self.ks + self.vel)))
        if(abs(delta_t) >= self.angle_limit): 
            delta_t = np.sign(a*x0 + b*y0 + c)*self.angle_limit

        # Reduction in velocity proportional to output steering angle
        # Basically the bot slows down to (self.min_vel_ratio * self.vel_max) velocity when the output steering angle reaches its maximum
        self.vel = self.vel_max*(1-(((1-self.min_vel_ratio)*abs(delta_t))/self.angle_limit))
        self.avg_vel = (self.avg_vel*self.count + self.vel)/(self.count+1)
        self.count+=1

        # Logs/Testing output
        print(f'\nParams: k = {self.k}, ks = {self.ks}, vel(max) = {self.vel_max}, vel(min) = {self.vel_max*self.min_vel_ratio}')
        print(f'Goal: {self.waypoints[self.goal]}')
        print(f'Velocity = {self.vel}, Average Velocity = {self.avg_vel}')
        print(f'Crosstrack Error = {e_t}, Heading Error = {psi_t*180/np.pi} deg')
        print(f'Output steering angle: {delta_t*180/np.pi}\n')

        # Publishing the steering angle and updated velocity
        msg:AckermannDriveStamped = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.drive.steering_angle = float(delta_t)
        msg.drive.steering_angle_velocity = float(0.0) # This means that the change in steering angle in the sim happens almost instantly after the message is received (No delay)
        msg.drive.speed = self.vel
        msg.drive.acceleration = float(0.0)
        msg.drive.jerk = float(0.0)
        self.publisher.publish(msg)
        delta_t = 0
    

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