import robot.abb_irc5 as irc5
from robot.globals import *
# from robot.globals import global_state
import time
import cv2

if __name__ == "__main__":
    global_state.motion = MotionState.FIND_TARGET
    global_state.set_target(ThreePointState.FIND_CENTER)

    import robot.sensors as sensors
    print("Connecting to sensors")
    sensors.connect_sensors()
    sensors.start_sensors()
    time.sleep(2)

    if not sensors.connection_status():
        print(sensors.connection_status())
        print("Failed to connect to sensors or robot.")
        exit(1)
    else:
        print("Successful Connections")

    last_time = time.perf_counter()

    try:
        while True:
            if show:
                with canvas_lock:
                    cv2.imshow("AprilTag Real-Time Map", sensors.canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == 13:  # 13 is the ASCII code for the Enter key
                global_state.set_target(ThreePointState.FIND_X)

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        print("Disconnecting from robot...")
        sensors.stop_sensors()