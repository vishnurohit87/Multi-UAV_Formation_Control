from Vehicle import Vehicle
import numpy as np
from helpers import *
import time
from geopy.distance import geodesic

''' Loosely based on the following paper: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6798711'''

class FormationFlying(object):
    def __init__(self, num_uavs: int, offsets: list, port: int, takeoff_altitude: int):
        self.num_uavs = num_uavs
        self.vehicles = []
        for i in range(num_uavs):
            self.vehicles.append(Vehicle(port + 10 * i))

        self.takeoff_altitude = takeoff_altitude
        # waypoints = [self.vehicles[0].read_global_position()] + waypoints
        # self.waypoint_list = interpolate_waypoints(waypoints, 100)
        # print("Waypoint List:", [(waypoint.lat, waypoint.lon) for waypoint in self.waypoint_list])
        self.max_velocity = 1  # Reduced for better control
        self.wp_radius = 0.5  # Adjusted for better tolerance
        self.formation_tolerance = 0.5  # Tolerance for formation achievement

        # Formation control gains
        self.K1 = np.array([[1.5, 0], [0, 1.5]])  # Reduced for less aggressive control
        self.K2 = np.array([[0.75, 0], [0, 0.75]])  # Reduced for less aggressive control
        self.damping = 0.3  # Damping factor

        # Define square formation offsets (in meters, relative to the formation center)
        self.formation_offsets = offsets
        
    def limit_vector(self, vector):
        if np.linalg.norm(vector) > self.max_velocity:
            vector = (vector / np.linalg.norm(vector)) * self.max_velocity
        return vector

    def calculate_control_input_global(self, current_pos, desired_pos, velocity):
        # Convert global positions to numpy arrays for easier manipulation
        current_pos = np.array([current_pos.lat, current_pos.lon])
        desired_pos = np.array([desired_pos.lat, desired_pos.lon])

        # Compute error in meters
        error_meters = np.array([
            geodesic((current_pos[0], current_pos[1]), (desired_pos[0], current_pos[1])).meters,
            geodesic((current_pos[0], current_pos[1]), (current_pos[0], desired_pos[1])).meters
        ])

        if desired_pos[0] < current_pos[0]:
            error_meters[0] = -error_meters[0]
        if desired_pos[1] < current_pos[1]:
            error_meters[1] = -error_meters[1]

        velocity = np.array(velocity)
        control_input_vel = self.K1 @ error_meters + self.K2 @ velocity - self.damping * velocity
        
        return self.limit_vector(control_input_vel)
    
    def initialize_formation(self, formation_center=None):
        print("Starting Mission!")

        for i in range(self.num_uavs): #Change Mode to GUIDED
            self.vehicles[i].change_mode('GUIDED')
            print(f"Vehicle {i} changed mode to GUIDED successfully!")

        takeoff_check_flag = True
        for i in range(self.num_uavs): # Arm and takeoff all the drones
            takeoff_check_flag &= self.vehicles[i].arm_and_takeoff(self.takeoff_altitude)
            print(f"Vehicle {i} armed and took off successfully!")

        if not takeoff_check_flag:
            print("Error in arming and taking off! Aborting mission!")
            return

        print("All vehicles in the air!")

        # Formation Initialization
        self.yaw = [0,0,0,0]
        # formation_center = self.vehicles[0].read_global_position()  # Use the first drone position as the formation center
        desired_positions = calculate_desired_positions_global(formation_center, self.formation_offsets)

        forming = True
        self.yaw_update_interval = 30  # Correct yaw every 30 seconds during waypoint following
        self.yaw_update_time = time.time()

        print("Initializing Formation!")
        print("Desired Positions:", [(desired_positions[i].lat, desired_positions[i].lon) for i in range(self.num_uavs)])
        while forming:
            forming = False
            for i in range(self.num_uavs):
                current_pos = self.vehicles[i].read_global_position()
                # print(f"Drone {i} Current Position: {current_pos.lat}, {current_pos.lon}")
                desired_pos = desired_positions[i]
                velocity = np.array(self.vehicles[i].read_local_velocity())

                self.yaw[i] = calculate_yaw_angle(current_pos, formation_center)
                control_input = self.calculate_control_input_global(current_pos, desired_pos, velocity[:2])
                self.vehicles[i].update_velocity_yaw(control_input[0], control_input[1], 0, self.yaw[i])
                
                # Debugging statements
                # print(f"Drone {i} Current Position: {current_pos.lat}, {current_pos.lon}")
                # print(f"Drone {i} Desired Position: {desired_pos.lat}, {desired_pos.lon}")
                # print(f"Drone {i} Control Input: {control_input}")

                # Check if the UAV is within the formation radius with tolerance
                distance_to_formation = geodesic((current_pos.lat, current_pos.lon), (desired_pos.lat, desired_pos.lon)).meters
                # print(f"Drone {i} Distance to Formation: {distance_to_formation}")
                if distance_to_formation > self.wp_radius + self.formation_tolerance:
                    forming = True

            time.sleep(0.1)
        print("Formation Achieved! Proceeding to Waypoints")

    def waypoint_following(self, waypoint):

        formation_center = waypoint
        desired_positions = calculate_desired_positions_global(formation_center, self.formation_offsets)

        reached_waypoint = False
        while not reached_waypoint:
            for i in range(self.num_uavs):
                current_pos = self.vehicles[i].read_global_position()
                desired_pos = desired_positions[i]
                velocity = np.array(self.vehicles[i].read_local_velocity())

                control_input = self.calculate_control_input_global(current_pos, desired_pos, velocity[:2])
                self.vehicles[i].update_velocity_yaw(control_input[0], control_input[1], 0, self.yaw[i])

                # Check if the UAV is within the waypoint radius
                distance_to_waypoint = geodesic((current_pos.lat, current_pos.lon), (desired_pos.lat, desired_pos.lon)).meters
                # print(f"Drone {i} Distance to Waypoint: {distance_to_waypoint}")
                if distance_to_waypoint < self.wp_radius:
                    reached_waypoint = True

            # Periodically update yaw
            if time.time() - self.yaw_update_time > self.yaw_update_interval:
                for i in range(self.num_uavs):
                    current_pos = self.vehicles[i].read_global_position()
                    self.yaw[i] = calculate_yaw_angle(current_pos, formation_center)
                    self.vehicles[i].update_yaw(self.yaw[i])
                self.yaw_update_time = time.time()

                time.sleep(0.1)

        time.sleep(0.1)

if __name__ == "__main__":  # For individual code testing purposes
    num_uavs = 4
    port = 14552
    takeoff_altitude = 10
    waypoints = [
        Waypoint(42.33306111, -71.08725555),
        Waypoint(42.33229332, -71.08712756),
        Waypoint(42.33302090, -71.08613127)
    ]

    formation_flying = FormationFlying(num_uavs, port, takeoff_altitude)
    formation_flying.initialize_formation()
    for waypoint in waypoints:
        formation_flying.waypoint_following(waypoint)
    print("Mission Completed!")
    