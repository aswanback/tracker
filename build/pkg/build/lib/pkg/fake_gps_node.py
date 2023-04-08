import rclpy
import rclpy.node as node
from sensor_msgs.msg import NavSatFix


class FakeGPSNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('fake_gps_node')
        self.pub = node.create_publisher(NavSatFix, 'global_position/raw/fix')

    def run(self):
        gps = NavSatFix()
        gps.latitude = 10
        gps.longitude = 10
        self.pub.publish()

def main():
    rclpy.init()
    node = FakeGPSNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()