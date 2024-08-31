import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

class GetWaypoints(Node):
    def __init__(self):
        super().__init__('get_waypoints')
        self.file = '/sim_ws/src/f1tenth_stanley_controller'


def main(args=None):
    rclpy.init(args=args)
    node = GetWaypoints()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
