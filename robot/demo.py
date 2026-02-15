import robot.abb_irc5 as irc5
import robot.sensors as sensors
from enum import Enum

class MotionState(Enum):
    INIT_POSE = 0
    TRACK_TARGET = 1

X_TARGET = 639.3
Y_TARGET = 61.79
Z_TARGET = -905.67

motion_state = MotionState.INIT_POSE
init_displacement = sensors.CorrectionState()

if __name__ == "__main__":
    # sensor_client = sensors.connect_sensors()
    # sensors.start_sensors()
    robot = irc5.connect_robot()
    irc5.read_robot_state()
    print(f"initial robot position: ({irc5.robot_state.pos[0]:.4f}, {irc5.robot_state.pos[1]:.4f}, {irc5.robot_state.pos[2]:.4f})")
    try:
        while motion_state == MotionState.INIT_POSE:
            init_displacement.dx = 20
            init_displacement.dy = 30
            init_displacement.dz = 200
            # print(f"Initial Displacement: ({init_displacement.dx:.4f}, {init_displacement.dy:.4f}, {init_displacement.dz:.4f})")
            dx = X_TARGET + init_displacement.dx
            dy = Y_TARGET - init_displacement.dy
            dz = Z_TARGET + init_displacement.dz
            irc5.move_robot_frame(dx, dy, dz)
            irc5.read_robot_state()
            print(f"Moving to Initial Pose: ({irc5.robot_state.pos[0]:.4f}, {irc5.robot_state.pos[1]:.4f}, {irc5.robot_state.pos[2]:.4f})")
            motion_state = MotionState.TRACK_TARGET

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        print("Disconnecting from robot...")
        irc5.stop_robot()
        irc5.disconnect_robot()
        # sensors.stop_sensors()