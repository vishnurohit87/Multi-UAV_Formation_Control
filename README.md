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

https://github.com/vishnurohit87/Multi-UAV_Formation_Control/assets/132724711/656dd02a-05e8-4505-9f80-3680da8143fa



### Notes
- The mavproxy port type can be changed in Vehicle.py
- The number of UAVs and formation offsets and port number can be determined in main.py
