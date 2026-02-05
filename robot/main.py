import robot.abb_irc5 as irc5
import robot.sensors as sensors
from enum import Enum

class MotionState(Enum):
    IDLE = 0
    FIND_CENTER = 1
    DESCEND = 2
    FIND_DEPTH = 3
    ASCEND = 4

if __name__ == "__main__":
    # sensor_client = sensors.connect_sensors()
    # sensors.start_sensors()
    robot = irc5.connect_robot()
    
    try:
        while True:
            irc5.read_robot_state()
            print(f"Current Position: {irc5.robot_state.pos} Orientation: {irc5.robot_state.orientation}")
            irc5.send_cartesian_command(10, 0, 0)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        irc5.disconnect_robot()
        # sensors.stop_sensors()