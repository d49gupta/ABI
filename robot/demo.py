import robot.abb_irc5 as irc5
import robot.sensors as sensors
from enum import Enum
import cv2

class MotionState(Enum):
    INIT_POSE = 0
    TRACK_TARGET = 1

# -- GLOBALS ---
# TODO: make config file for all globals
X_TARGET = 639.3
Y_TARGET = 61.79
Z_TARGET = -905.67
PENCIL_Z_OFFSET = 100 # mm

motion_state = MotionState.INIT_POSE
init_displacement = sensors.CorrectionState()

if __name__ == "__main__":
    sensor_client = sensors.connect_sensors()
    sensors.start_sensors()
    robot = irc5.connect_robot()
    irc5.read_robot_state()
    print(f"initial robot position: ({irc5.robot_state.pos[0]:.4f}, {irc5.robot_state.pos[1]:.4f}, {irc5.robot_state.pos[2]:.4f})")
    try:
        while motion_state == MotionState.INIT_POSE:
            cv2.imshow("AprilTag Real-Time Map", sensors.canvas)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            dx = X_TARGET + sensors.correction.dx
            dy = Y_TARGET - sensors.correction.dy
            dz = Z_TARGET + sensors.correction.est_z - PENCIL_Z_OFFSET # TODO: NEED to properly account for height diff between TCP and camera)
            irc5.move_robot_frame(dx, dy, dz)
            irc5.read_robot_state()

            if abs(dx*dx + dy*dy) < 1.0:
                sensors.camera_logger.info("Target Reached")
                irc5.robot_logger.info("Target Reached")
                break
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        print("Disconnecting from robot...")
        irc5.stop_robot()
        irc5.disconnect_robot()
        sensors.stop_sensors()