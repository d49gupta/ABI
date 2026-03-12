import robot.abb_irc5 as irc5
import robot.sensors as sensors
import cv2
from robot.globals import *
import time
import robot.main as main

if __name__ == "__main__":
    print("Connecting to sensors")
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
    else:
        print("Successful Connections")

    last_time = time.perf_counter()
    main.motion_state = MotionState.FIND_CENTER
    try:
        while True:
            if main.motion_state == MotionState.IDLE:
                break

            if not sensors.connection_status() or not irc5.connection_status():
                print("Lost connection to sensors or robot.")
                break
            
            if not sensors.correction_buffer:
                correction_logger.warning("No correction data available yet.")
                continue

            if not irc5.robot_pose_buffer:
                continue
            
            if show:
                with canvas_lock:
                    cv2.imshow("AprilTag Real-Time Map", sensors.canvas)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if pencil_buffer and pencil_buffer[-1].active:
                if main.motion_state != MotionState.FIND_DEPTH:
                    main.motion_state = MotionState.FIND_DEPTH
                    print("Pencil Detected. Switching to FIND_DEPTH mode.")
                    time.sleep(5)
                
            correction = sensors.correction_buffer[-1]
            dx = X_TARGET + correction.dx
            dy = Y_TARGET - correction.dy
            dz = Z_TARGET + correction.dz - PENCIL_Z_OFFSET # TODO: NEED to properly account for height diff between TCP and camera

            current_time = time.perf_counter()
            if current_time - last_time < ROBOT_PUBLISH_RATE:
                continue

            last_time = current_time
            irc5.move_robot_frame(dx, dy, dz)
            main.state_machine()

            dx_diff = irc5.robot_state.pos[0] - X_TARGET
            dy_diff = Y_TARGET - irc5.robot_state.pos[1]
            dz_diff = irc5.robot_state.pos[2] - Z_TARGET

            correction_logger.info("%d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", main.motion_state.value, correction.dx, 
                             correction.dy, correction.dz, dx_diff, dy_diff, dz_diff)

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        print("Disconnecting from robot...")
        irc5.stop_robot()
        irc5.stop_reading_robot()
        irc5.disconnect_robot()
        sensors.stop_sensors()