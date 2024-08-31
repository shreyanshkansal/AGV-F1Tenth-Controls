import rclpy
import csv
from rclpy.node import Node
from nav_msgs.msg import Odometry
import rclpy.time

class GetWaypoints(Node):
    def __init__(self):
        super().__init__('get_waypoints')
        self.file = open('/sim_ws/src/f1tenth_stanley_controller/resource/waypoints.csv',mode='w')
        self.subscriber = self.create_subscription(Odometry,'/ego_racecar/odom',250,self.OdomCalback)
        self.count = 0
        
    def OdomCalback(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        waypoint = [x,y]
        writer = csv.writer(self.file)
        writer.writerow(waypoint)
        self.count+=1
        print(f'Waypoint {self.count} successfully stored!')

def main(args=None):
    rclpy.init(args=args)
    node = GetWaypoints()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
