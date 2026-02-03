import robot.abb_irc5 as irc5
import robot.sensors as sensors
from enum import Enum

class MotionState(Enum):
    IDLE = 0
    FIND_CENTER = 1
    DESCEND = 2
    FIND_DEPTH = 3
    ASCEND = 4

TARGET_X = 600
TARGET_Y = 0
TARGET_Z = 750

if __name__ == "__main__":
    sensor_client = sensors.connect_sensors()
    sensors.start_sensors()
    robot = irc5.connect_robot()
    
    try:
        while True:
            irc5.receive_data()
            print(f"Current Position: {irc5.robot_state.pos}")
            dx = TARGET_X - irc5.robot_state.pos[0]
            dy = TARGET_Y - irc5.robot_state.pos[1]
            dz = TARGET_Z - irc5.robot_state.pos[2]
            irc5.send_cartesian_command(dx, dy, dz, 1.0, 0.0, 0.0, 0.0)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        sensors.stop_sensors()