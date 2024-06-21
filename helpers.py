from geopy import distance
from geopy.distance import geodesic
import math

class Waypoint:
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon
        self.alt = 0

def calculate_distance_lla(pos1, pos2):
    # Calculate the distance between two points in meters
    try:
        return distance.distance((pos1.lat, pos1.lon), (pos2.lat, pos2.lon)).m
    except:
        return distance.distance((pos1.latitude_deg, pos1.longitude_deg), (pos2.latitude_deg, pos2.longitude_deg)).m

def calculate_desired_positions_global(formation_center, offsets):
    desired_positions = []
    for offset in offsets:
        # Calculate the new latitude and longitude based on the offset
        new_position = geodesic(meters=offset[0]).destination((formation_center.lat, formation_center.lon), 0)  # Offset in the north direction
        new_lat = new_position.latitude
        new_position = geodesic(meters=offset[1]).destination((new_lat, formation_center.lon), 90)  # Offset in the east direction
        new_lon = new_position.longitude
        desired_positions.append(Waypoint(new_lat, new_lon))
    return desired_positions

def interpolate_waypoints(waypoints: list, num_points: int):
    '''
    Interpolate between the waypoints given in the list to generate a new waypoint_list.
    waypoints: List of waypoints to interpolate between.
    num_points: Number of points to interpolate between two consecutive waypoints.
    '''
    interpolated_waypoints = []
    for i in range(len(waypoints) - 1):
        lat_diff = (waypoints[i + 1].lat - waypoints[i].lat) / num_points
        lon_diff = (waypoints[i + 1].lon - waypoints[i].lon) / num_points

        for j in range(num_points):
            interpolated_waypoints.append(Waypoint(waypoints[i].lat + j * lat_diff, waypoints[i].lon + j * lon_diff))
    interpolated_waypoints.append(waypoints[-1])
    return interpolated_waypoints

def calculate_yaw_angle(current_pos, center_pos):
    delta_y = center_pos.lat - current_pos.lat
    delta_x = center_pos.lon - current_pos.lon
    yaw = math.atan2(delta_y, delta_x)
    if delta_x*delta_y < 0:
        yaw = (math.pi/2)-yaw
    return yaw