from subscriber import GPSSubscriber, ros_spin
from helpers import Waypoint
import time
from threading import Thread
from FFC_centroid_based import FormationFlying
import rclpy

def main(args=None):
    rclpy.init(args=args)

    gps_subscriber = GPSSubscriber()
    formation_flying = FormationFlying(num_uavs=4, port=14552, takeoff_altitude=10)

    # Spin the subscriber in a separate thread
    spin_thread = Thread(target=ros_spin, args=(gps_subscriber,))
    spin_thread.start()

    try:
        formation_flag = True
        while rclpy.ok():
            gps_data = gps_subscriber.get_gps_data()
            
            if gps_data.lat == 0 and gps_data.lon == 0:
                continue

            if formation_flag:

                formation_flying.initialize_formation(gps_data)
                formation_flag = False

            else:
                formation_flying.waypoint_following(gps_data)
            time.sleep(0.1) # Frequency of receiving waypoints

    except KeyboardInterrupt:
        print("Keyboard Interrupt Detected! Exiting...")

    rclpy.shutdown()
    spin_thread.join()
    gps_subscriber.destroy_node()

if __name__ == '__main__':
    main()