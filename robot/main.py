import robot.abb_irc5 as irc5
import robot.sensors as sensors
from enum import Enum
from scripts.logger import CSVLogger

class MotionState(Enum):
    IDLE = 0
    FIND_CENTER = 1
    DESCEND = 2
    FIND_DEPTH = 3
    ASCEND = 4

X_TARGET = 756.827
Y_TARGET = 77.41
Z_TARGET = -905.67
Z_THRESH = 4.0

final_robot_pose = None
motion_state = MotionState.IDLE

def move_xy_sensors():
    dx = sensors.correction.dx
    dy = sensors.correction.dy
    magnitude = (dx**2 + dy**2)**0.5

    if magnitude > 1.0:
        dx_norm = dx / magnitude
        dy_norm = dy / magnitude
    
    irc5.robot_logger.info("%.4f, %.4f, %.4f", dx, dy, 0)
    irc5.send_cartesian_command(dx_norm, dy_norm, 0)

def move_xyz_sensors():
    # dx and dy magnitude should be less than 1.0
    dx = sensors.correction.dx
    dy = sensors.correction.dy
    dz = 0.0

    if not sensors.correction.active_dz or abs(sensors.correction.dz) < Z_THRESH:
        dz = 1.0
    
    irc5.robot_logger.info("%.4f, %.4f, %.4f", dx, dy, dz)
    irc5.send_cartesian_command(dx, dy, dz)

def move_xy_target():
    global motion_state
    dx = X_TARGET - irc5.robot_state.pos[0]
    dy = Y_TARGET - irc5.robot_state.pos[1]
    magnitude = (dx**2 + dy**2)**0.5

    if magnitude > 1.0:
        dx_norm = dx / magnitude
        dy_norm = dy / magnitude
        irc5.robot_logger.info("%.4f, %.4f, %.4f", dx, dy, 0)
        irc5.send_cartesian_command(dx_norm, dy_norm, 0)
    else:
        print(f"Center Target Reached: ({irc5.robot_state.pos[0]:.4f}, {irc5.robot_state.pos[1]:.4f})")
        irc5.robot_logger.info("Center Target Reached (%.4f, %.4f)", irc5.robot_state.pos[0], irc5.robot_state.pos[1])
        motion_state = MotionState.DESCEND
        return

def move_xyz_target():
    global motion_state
    # dx and dy magnitude should be less than 1.0
    dx = X_TARGET - irc5.robot_state.pos[0]
    dy = Y_TARGET - irc5.robot_state.pos[1]
    dz = Z_TARGET - irc5.robot_state.pos[2]
    magnitude = (dx**2 + dy**2 + dz**2)**0.5

    if magnitude > 1.0:
        dx_norm = dx / magnitude
        dy_norm = dy / magnitude
        dz_norm = dz / magnitude
    else:
        print(f"Final Target Reached: ({irc5.robot_state.pos[0]:.4f}, {irc5.robot_state.pos[1]:.4f}, {irc5.robot_state.pos[2]:.4f})")
        irc5.robot_logger.info("Final Target Reached: (%.4f, %.4f, %.4f)", irc5.robot_state.pos[0], irc5.robot_state.pos[1], irc5.robot_state.pos[2])
        motion_state = MotionState.ASCEND
        return

    irc5.robot_logger.info("%.4f, %.4f, %.4f", dx, dy, dz)
    irc5.send_cartesian_command(dx_norm, dy_norm, dz_norm)

def ascent():
    global motion_state
    dz = irc5.robot_state.initial_pos[2] - irc5.robot_state.pos[2]
    if abs(dz) < 10.0:
        print("Ascent Complete")
        irc5.robot_logger.info("Ascent Complete")
        motion_state = MotionState.IDLE

    irc5.robot_logger.info("%.4f, %.4f, %.4f", 0, 0, dz)
    irc5.send_cartesian_command(0, 0, 1)

if __name__ == "__main__":
    sensor_client = sensors.connect_sensors()
    sensors.start_sensors()
    robot = irc5.connect_robot()
    motion_state = MotionState.FIND_CENTER
    
    try:
        while motion_state == MotionState.FIND_CENTER:
            irc5.read_robot_state()
            print(f"Current Position: {irc5.robot_state.pos}, Orientation: {irc5.robot_state.orientation}")
            move_xy_target()
        while motion_state == MotionState.DESCEND:
            irc5.read_robot_state()
            print(f"Current Position: {irc5.robot_state.pos}, Orientation: {irc5.robot_state.orientation}")
            move_xyz_target()
        while motion_state == MotionState.ASCEND:
            irc5.read_robot_state()
            print(f"Current Position: {irc5.robot_state.pos}, Orientation: {irc5.robot_state.orientation}")
            ascent()

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        print("Disconnecting from robot...")
        irc5.stop_robot()
        irc5.disconnect_robot()
        sensors.stop_sensors()