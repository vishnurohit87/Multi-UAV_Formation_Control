# Multi-UAV_Formation_Control
A multi-uav formation control algorithm for tracking a dynamic target. Developed for multirotors running on ArduCopter, this system uses ROS2 to get GPS coordinates of the target and mavlink is used for communicating with the UAVs.

### Requirements
- ardupilot
- ROS2
- Python3
- pymavlink
- geopy
- numpy

### Results
The following results are from a simulation demo on the SITL simulator where vehicles 1-4 were used to make a square formation, and vehicle 5 was used as a dynamic target for the simulation to follow.

Note: The following demo is of an older version of the codee and does not show inter-UAV collision avoidance in action (which is now included in the code)

https://github.com/vishnurohit87/Multi-UAV_Formation_Control/assets/132724711/d1a21d8c-f9ef-4413-a08e-83112557dce2

### Notes
- The mavproxy port type can be changed in Vehicle.py
- The number of UAVs and formation offsets and port number can be determined in main.py
