import robot.abb_irc5 as irc5
import robot.sensors as sensors
import cv2
from robot.globals import *
import time

if __name__ == "__main__":
    sensors.connect_sensors()
    sensors.start_sensors()
    print("Connecting to robot...")
    irc5.connect_robot()
    irc5.start_reading_robot()
    time.sleep(2)

    if not sensors.connection_status() or not irc5.connection_status():
        print(sensors.connection_status(), irc5.connection_status())
        print("Failed to connect to sensors or robot.")
        exit(1)

    if not irc5.connection_status():
        print(irc5.robot_config.connected, irc5.robot_config.msg_count)
        print("Failed to connect to robot.")
        exit(1)

    counter = 0
    try:
        while True:
            if not sensors.connection_status() or not irc5.connection_status():
                print("Lost connection to sensors or robot.")
                break
            
            if not sensors.correction_buffer:
                correction_logger.warning("No correction data available yet.")
                continue

            if not irc5.robot_pose_buffer:
                irc5.robot_logger.warning("No robot pose data available yet.")
                continue

            with canvas_lock:
                cv2.imshow("AprilTag Real-Time Map", sensors.canvas)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if pencil_buffer and pencil_buffer[-1].active:
                print("PENCIL ACTIVE")
                break

            correction = sensors.correction_buffer[-1]
            dx = X_TARGET + correction.dx
            dy = Y_TARGET - correction.dy
            dz = Z_TARGET + correction.dz # TODO: NEED to properly account for height diff between TCP and camera
            irc5.move_robot_frame(dx, dy, dz)
            irc5.read_robot_state()

            dx_diff = irc5.robot_state.pos[0] - X_TARGET
            dy_diff = Y_TARGET - irc5.robot_state.pos[1]
            dz_diff = irc5.robot_state.pos[2] - Z_TARGET

            counter += 1
            correction_logger.info("%d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", counter, correction.dx, 
                             correction.dy, correction.dz, dx_diff, dy_diff, dz_diff)
            
            time.sleep(0.1) # cant send commands too fast
            # probably move this wait into move_robot_frame and any other send commands

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        print("Disconnecting from robot...")
        irc5.stop_robot()
        irc5.stop_reading_robot()
        irc5.disconnect_robot()
        sensors.stop_sensors()