from pymavlink import mavutil
import time


class Vehicle:
    def __init__(self, port):
        self.master = mavutil.mavlink_connection(f'udpin:localhost:{port}')
        self.master.wait_heartbeat()

        print("Heartbeat from system (system %u component %u)" % 
            (self.master.target_system, self.master.target_component))

    def read_gps(self):
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        return msg

    def read_attitude(self):
        msg = self.master.recv_match(type='ATTITUDE', blocking=True)
        return msg
    
    def read_local_position(self):
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        return msg
    
    def read_global_position(self):
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        msg.lat = msg.lat*1e-7
        msg.lon = msg.lon*1e-7
        return msg

    def read_local_velocity(self):
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        return [msg.vx, msg.vy, msg.vz]
    
    def read_global_velocity(self):
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        msg.vx = msg.vx*1e-2
        msg.vy = msg.vy*1e-2
        msg.vz = msg.vz*1e-2
        return [msg.vx, msg.vy, msg.vz]

    def read_heartbeat(self):
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
        return msg
    
    def change_mode(self, mode):
        print(f"Changing mode to - {mode}!")
        # print("Test")
        while True:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system, 
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
                    0, 
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 
                    self.master.mode_mapping()[mode], 
                    0, 0, 0, 0, 0)

                msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
                
                # print(f"ACK msg: {msg.result}")
                return msg.result == 0
            
            except:
                continue
        

    def arm(self, forced=False):
        p2 = 2989 if forced else 0
        self.master.mav.command_long_send(
                self.master.target_system,  # target_system
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
                0, # confirmation
                1, # param1 (1 to indicate arm)
                p2, # param2  (all other params meaningless)
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7

        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
        return msg.result == 0
    
    def disarm(self, forced=False):
        p2 = 21196 if forced else 0
        self.master.mav.command_long_send(
                self.master.target_system,  # target_system
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
                0, # confirmation
                0, # param1 (1 to indicate arm)
                p2, # param2  (all other params meaningless)
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7

        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        return msg.result == 0

    def takeoff(self, altitude):
        self.master.mav.command_long_send(
            self.master.target_system, 
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude)

        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        return msg.result == 0

    def land(self):
        self.master.mav.command_long_send(
            self.master.target_system, 
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0)

        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        return msg.result == 0
    
    def update_position(self, x, y, z):
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,
            x, y, z,
            0, 0, 0,
            0, 0, 0,
            0, 0)
    
    def update_yaw(self, yaw):
        self.master.mav.command_long_send(
            self.master.target_system, 
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0, 
            yaw, 35, 0, 0, 0, 0, 0)

        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        return msg.result == 0
    
    def update_velocity_yaw(self, vx, vy, vz, yaw): # NED
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000100111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            yaw, 1.57)
        
    def update_speed(self, speed): # Only ground speed for now. Can be modified
        self.master.mav.command_long_send(
                self.master.target_system,  # target_system
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, # command
                0, # confirmation
                1, # param1 (1 to indicate Ground Speed)
                1, # param2  (speed in m/s. All other params meaningless)
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7

        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
        return msg.result == 0

    def update_acceleration(self, ax, ay, az):
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b110000111111,
            0, 0, 0,
            0, 0, 0,
            ax, ay, az,
            0, 0)

    def update_altitude(self, altitude):    # TODO Check this function
        print(f"Changing altitude to - {altitude} meters!")
        # Assuming self.master is the MAVLink connection
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_ALTITUDE,
            0,  # Confirmation field
            0,  # Param1: Hold time in decimal seconds (ignored in this case)
            altitude,  # Param2: Altitude in meters
            0, 0, 0, 0, 0  # Param3 to Param7 (unused in this case)
        )
        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        return msg.result == 0
        
    def reach_altitude(self, altitude):
        while True:
            # TODO Change the update call once the update_altitude function is implemented
            curr_altitude = -self.read_local_position().z
            curr_x = -self.read_local_position().x
            curr_y = -self.read_local_position().y
            self.update_position(curr_x, curr_y, altitude)
            time.sleep(3)
            if curr_altitude >= altitude*0.95 and curr_altitude <= altitude*1.05:
                print("Reached target altitude!")
                break
    
    def check_arm(self):
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
        return msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED != 0

    def check_mode(self):
        #TODO: Check if this is the correct way to get the mode
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
        return msg.custom_mode
    
    def arm_and_takeoff(self, altitude):  # TODO check logic of takeoff
        # Check if the drone is already armed or not
        already_armed = self.check_arm()

        # Check if the drone is in GUIDED mode or not
        already_guided = self.check_mode() == self.master.mode_mapping()['GUIDED']
        if not already_guided:
            print("Vehicle is not in GUIDED mode!")
            print("Aborting takeoff!")
            return False
        
        # Check if the drone is already in the air or not
        already_in_air = self.read_local_position().z < -1.0
        print(f"Already in air: {already_in_air}\nCurrent altitude: {-self.read_local_position().z}\nTarget altitude: {altitude}")

        if not already_armed:
            arm_flag = self.arm()
            if not arm_flag:
                arm_flag = self.arm(True)
                print("Trying to arm the drone!")
            else:
                print("Vehicle armed successfully!")
        else:
            print("Vehicle already armed!")

        if not already_in_air:
            takeoff_flag = self.takeoff(altitude)
            if not takeoff_flag:
                takeoff_flag = self.takeoff(altitude)
                print("Trying to takeoff the drone!")
            else:
                print("Vehicle took off successfully!")
                # Run a loop till the drone reaches the desired altitude
                while True:
                    curr_altitude = -self.read_local_position().z
                    print(f"Altitude: {curr_altitude}")
                    if curr_altitude >= altitude*0.95:
                        print("Reached target altitude!")
                        break
                    time.sleep(1)
        else:
            print("Vehicle already in the air!")
        
        return True 
                               



if __name__ == "__main__":      # Test code
    v = Vehicle(14551)
    # while True:
    #     print(swarm.read_gps())
    #     print(swarm.read_attitude())
    #     print(swarm.read_local_position())
    #     print(swarm.read_global_position())
    #     print(swarm.read_heartbeat())
    #     time.sleep(1)

    # print(v.master.mode_mapping())

    print(type(v.read_local_position()))
    # v.change_mode('GUIDED')
    # v.arm_and_takeoff(5)
    # v.read_local_position()
    
    # for i in range(10):
    #     v.update_velocity(1, 0, 0)
    #     print(v.read_local_position())
    #     print(v.read_global_position())
    #     time.sleep(1)

    # v.land()

