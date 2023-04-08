import math
import rclpy
import rclpy.node
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')

# I don't understand ROS2 using another python module sorry this is the cleanest I've got
import sys
sys.path.append('/root/ws/src/tracker_pkg/tracker_pkg/accel_stepper.py')
from tracker_pkg.accel_stepper import AccelStepper, MotorInterfaceType


class Loc():
    lat: float
    lon: float

    def __init__(self, lat:float, lon:float) -> None:
        self.lat = lat
        self.lon = lon

# NOTES
# Stepper initializes to a zero position that this code assumes is North
# Antenna must be faced north prior to power on
class TrackerNode(rclpy.node.Node):
    # None for unknown value
    uav_loc:Loc|None = None
    at_loc:Loc|None = None
    
    def __init__(self, debug=False, vis=False):
        super().__init__('tracker_node')
        # debug
        self.log = self.get_logger().info
        self.debug = debug
        self.vis = vis

        self.desired_abs_heading = 0 

        # get positions
        self.create_subscription(NavSatFix, '/global_position/raw/fix', self.uav_pos_callback, 10)
        self.create_subscription(NavSatFix, '/tracker_pos', self.at_pos_callback, 10)

        direction_pin = 16                      # hardware defined
        step_pin = 18                           # hardware defined
        type = MotorInterfaceType.FULL2WIRE     # hardware defined, microstepping unsupported
        self.steps_per_revolution = 200         # hardware defined, microstepping unsupported
        self.stepper = AccelStepper(type, direction_pin, step_pin)
        self.stepper.setMaxSpeed(1000)
        self.stepper.setAcceleration(50)
        self.create_timer(0.001, self.stepper_run_callback)
        
        if vis: 
            self.display()
    
    def stepper_run_callback(self):
        # actually moves the stepper, has to be called continuously
        self.stepper.run()
        if self.vis:
            self.update_display()

    def uav_pos_callback(self,data):
        self.uav_loc = Loc(data.latitude, data.longitude)
        if self.debug: self.log(f'uav {self.uav_loc.lon}, {self.uav_loc.lat}')
        self.recalculate() 
    
    def at_pos_callback(self,data):
        self.at_loc = Loc(data.latitude, data.longitude)
        if self.debug: self.log(f'at {self.at_loc.lon}, {self.at_loc.lat}')
        self.recalculate()
    
    def steps_to_radians(self,steps):
        return steps / self.steps_per_revolution * 2*math.pi
    def radians_to_steps(self, radians):
        return radians / (2*math.pi) * self.steps_per_revolution

    def recalculate(self):
        if self.uav_loc is None or self.at_loc is None:
            return
        self.desired_abs_heading = math.atan2(self.uav_loc.lon - self.at_loc.lon, self.uav_loc.lat - self.at_loc.lat)
        desired_rel_heading = self.desired_abs_heading - self.steps_to_radians(self.stepper._currentPos)
        desired_rel_heading = (desired_rel_heading + math.pi) % (2 * math.pi) - math.pi # keeps from flipping -179.99 to 179.99 at 180deg boundary
        steps = self.radians_to_steps(desired_rel_heading)
        self.stepper.move(steps) # negative becuase positive radians are CCW, positive steps are CW

    # visualization for validation
    def display(self):
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)

        lon,lat = -86.913871, 40.433261
        plot_size = 1/69 # one mile is 1/69 in fractional gps coords
        self.ax.set_xlim([lon-plot_size, lon+plot_size])
        self.ax.set_ylim([lat-plot_size, lat+plot_size])

        self.uav_scatter = self.ax.scatter([], [],color='b')          # uav location
        self.at_scatter = self.ax.scatter([], [],color='g')           # antenna tracker location
        self.curr_head_arrow = self.ax.arrow(0, 0, 0, 0)    # current heading
        self.des_head_arrow = self.ax.arrow(0, 0, 0, 0)     # desired heading
    
    def update_display(self):
        if self.at_loc is not None:
            self.at_scatter.set_offsets([[self.at_loc.lon, self.at_loc.lat]])
        if self.uav_loc is not None:
            self.uav_scatter.set_offsets([[self.uav_loc.lon, self.uav_loc.lat]])
        if self.uav_loc is not None and self.at_loc is not None:
            # desired heading
            length = 0.005
            self.des_head_arrow.remove()
            self.des_head_arrow = self.ax.arrow(self.at_loc.lon, self.at_loc.lat, length*math.sin(self.desired_abs_heading), length*math.cos(self.desired_abs_heading), width=0.00005, color='b')
            # current heading
            length = 0.0025
            self.curr_head_arrow.remove()
            current_heading = self.steps_to_radians(self.stepper._currentPos)
            self.curr_head_arrow = self.ax.arrow(self.at_loc.lon, self.at_loc.lat, length*math.sin(current_heading), length*math.cos(current_heading), width=0.0001, color='g')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    

def main():
    rclpy.init()
    node = TrackerNode(debug=False, vis=True)
    rclpy.spin(node)

if __name__ == '__main__':
    main()