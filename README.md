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

Velocity control **without inter-UAV collision avoidance** component:


https://github.com/user-attachments/assets/eeb27a01-758e-4a4e-9096-8818c5d736d9



Velocity control **with inter-UAV collision avoidance** component:

https://github.com/user-attachments/assets/ae3d65eb-2f08-47a3-9d37-430106b90164


### Notes
- The mavproxy port type can be changed in Vehicle.py
- The number of UAVs and formation offsets and port number can be determined in main.py
