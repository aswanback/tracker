import rclpy
import rclpy.node as node
from sensor_msgs.msg import NavSatFix

class TrackerNode(rclpy.node.Node):
    
    def __init__(self):
        super().__init__('tracker_node')
        self.log = self.get_logger().info
        self.create_subscription(NavSatFix, '/global_position/raw/fix', self.gps_callback, 10)

    def gps_callback(self,data):
        self.log(data)

def main():
    rclpy.init()
    node = TrackerNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()