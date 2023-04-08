import rclpy
import rclpy.node as node
from sensor_msgs.msg import NavSatFix

class FakeGPSNode(rclpy.node.Node):
    last_gps = None
    lat_incr = 0.0001
    long_incr = 0.0001

    def __init__(self):
        super().__init__('fake_gps_node')
        self.log = self.get_logger().info
        self.pub = self.create_publisher(NavSatFix, 'global_position/raw/fix', 1)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.last_gps = NavSatFix()
        self.last_gps.latitude = 40.433261
        self.last_gps.longitude = -86.913871

    def timer_callback(self):
        gps = NavSatFix()
        gps.latitude = self.last_gps.latitude + self.lat_incr
        gps.longitude = self.last_gps.longitude + self.long_incr
        self.pub.publish(gps)


def main():
    rclpy.init()
    node = FakeGPSNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()