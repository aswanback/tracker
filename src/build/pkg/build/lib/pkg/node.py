import rclpy
import rclpy.node as node
from sensor_msgs.msg import NavSatFix


class TrackerNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('tracker_node')
        node.create_subscriber(NavSatFix ,'global_position/raw/fix', self.gps_callback)

    def gps_callback(self, data):
        print(data)
        pass


def main():
    rclpy.init()
    node = TrackerNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()