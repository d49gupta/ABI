import robot.abb_irc5 as irc5
import robot.sensors as sensors
from enum import Enum
from scripts.logger import CSVLogger

test_logger = CSVLogger(name="diff", log_dir="test_logs")

X_TARGET = 750
Y_TARGET = 0.0
Z_TARGET = -905.0
Z_THRESH = 4.0

def move_xy():
    dx = sensors.correction.dx
    dy = sensors.correction.dy
    magnitude = (dx**2 + dy**2)**0.5

    if magnitude > 1.0:
        dx_norm = dx / magnitude
        dy_norm = dy / magnitude
    
    test_logger.info("%.4f, %.4f, %.4f", dx, dy, 0)
    irc5.send_cartesian_command(dx_norm, dy_norm, 0)

def move_xyz():
    # dx and dy magnitude should be less than 1.0
    dx = sensors.correction.dx
    dy = sensors.correction.dy
    dz = 0.0

    if not sensors.correction.active_dz or abs(sensors.correction.dz) < Z_THRESH:
        dz = 1.0
    
    test_logger.info("%.4f, %.4f, %.4f", dx, dy, dz)
    irc5.send_cartesian_command(dx, dy, dz)

if __name__ == "__main__":
    # sensor_client = sensors.connect_sensors()
    # sensors.start_sensors()
    robot = irc5.connect_robot()
    
    try:
        while True:
            irc5.read_robot_state()
            print(f"Current Position: {irc5.robot_state.pos}, Orientation: {irc5.robot_state.orientation}")
            # print(f"Initial Position: {irc5.robot_state.initial_pos}")
            dx = X_TARGET - irc5.robot_state.pos[0]
            dy = Y_TARGET - irc5.robot_state.pos[1]
            dz = Z_TARGET - irc5.robot_state.pos[2]
            magnitude = (dx**2 + dy**2 + dz**2)**0.5

            if magnitude <= 1.0:
                print("Target reached.")
                break

            dx_norm = dx / magnitude
            dy_norm = dy / magnitude
            dz_norm = dz / magnitude

            test_logger.info("%.4f, %.4f, %.4f", dx, dy, dz)
            irc5.send_cartesian_command(dx_norm, dy_norm, dz_norm)

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        irc5.disconnect_robot()
        # sensors.stop_sensors()