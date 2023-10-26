# tracker

This is a full ROS node created to implement an Antenna Tracker for the Purdue Aerial Robotics Team. The physical construction of the antenna tracker is a geared stepper motor on a platform, rotating an antenna on top. The hardware supports infinite rotation due to a slip ring but the software supports both infinite and bounded rotations. 

The tracker recieves the UAV's GPS coordinates from another (arbitrary) ROS node and calculates based on its own position (via onboard GPS) where to point. Though the antenna can get some reception at off-angles, the highest Mbps rate is achieved by following the UAV so the tracker was necessary.

The code includes a simulation that proves the system works as intended, and a real-world test at our airfield was succesful in transmission speeds above 10Mbps while the UAV was in flight.
