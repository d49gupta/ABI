import robot.abb_irc5 as irc5
import robot.sensors as sensors
import cv2
from robot.globals import *
import time

if __name__ == "__main__":
    sensor_client = sensors.connect_sensors()
    sensors.start_sensors()
    robot = irc5.connect_robot()
    time.sleep(2)

    if not sensors.connection_status() or not irc5.connection_status():
        print(sensors.connection_status(), irc5.connection_status())
        print("Failed to connect to sensors or robot.")
        exit(1)

    irc5.read_robot_state()
    print(f"initial robot position: ({irc5.robot_state.pos[0]:.4f}, {irc5.robot_state.pos[1]:.4f}, {irc5.robot_state.pos[2]:.4f})")
    counter = 0
    try:
        while True:
            cv2.imshow("AprilTag Real-Time Map", sensors.canvas)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            dx = X_TARGET + sensors.correction.dx
            dy = Y_TARGET - sensors.correction.dy
            dz = Z_TARGET + sensors.correction.est_z # TODO: NEED to properly account for height diff between TCP and camera)
            irc5.move_robot_frame(dx, dy, dz)
            irc5.read_robot_state()

            dx_diff = irc5.robot_state.pos[0] - X_TARGET
            dy_diff = Y_TARGET - irc5.robot_state.pos[1]
            dz_diff = irc5.robot_state.pos[2] - Z_TARGET

            counter += 1
            diff_logger.info("%d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", counter, sensors.correction.dx, 
                             sensors.correction.dy, sensors.correction.est_z, dx_diff, dy_diff, dz_diff)

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