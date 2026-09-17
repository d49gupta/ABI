import robot.abb_irc5 as irc5
from robot.globals import *
import time
import cv2
import sys
import math

RUN_MODE = "full" # "three", "four", or "full" -- set from argv in __main__

def set_robot_speed(speed):
    global global_state
    if global_state.robot_config.tcp_speed != speed:
        irc5.set_speed(speed)
        global_state.robot_config.tcp_speed = speed

def find_target():
    global global_state, conveyor_state

    if not sensors.correction_buffer:
        event_logger.warning("Not enough correction data for camera smoothing.")
        return
    
    global smooth_dx, smooth_dy
    camera_curr_correction = sensors.correction_buffer[-1]
    smooth_dx = (alpha_camera * camera_curr_correction.dx) + (1 - alpha_camera) * smooth_dx
    smooth_dy = (alpha_camera * camera_curr_correction.dy) + (1 - alpha_camera) * smooth_dy
    magnitude = (smooth_dx**2 + smooth_dy**2)**0.5

    curr_time = time.perf_counter()
    if conveyor_state.running and curr_time - conveyor_state.last_time >= CONVEYOR_MOVE_TIME:
        if global_state.calibration == CalibrationMode.FOUR_POINT:
            print("Stopping the conveyor")
            event_logger.info("Stopping the conveyor")    
            irc5.stop_conveyor()
            conveyor_state.running = False
            conveyor_state.last_time = None

    if magnitude > XY_TARGET_ACC or conveyor_state.running:
        dx = Kp_target * smooth_dx
        dy = Kp_target * smooth_dy

        correction_magnitude = math.sqrt(smooth_dx*smooth_dx + smooth_dy*smooth_dy)
        t = max(0, min(1, correction_magnitude / 100))
        speed = DESCENT_SPEED + int((FIND_TARGET_SPEED - DESCENT_SPEED) * t)
        set_robot_speed(speed)

        controller_logger.info("%d, %.4f, %.4f, %.4f, %.4f", global_state.motion.value, global_state.robot_config.tcp_speed, dx, dy, 0.0)
        irc5.move_rel_frame(dx, dy, 0.0)
    else:
        print(f"Camera Correction Target Reached")
        event_logger.info("Camera Correction Target Reached")
        smooth_dx = 0
        smooth_dy = 0
        global_state.motion = MotionState.DESCEND

def descend():
    # Main loop will trigger pencil interrupt to go into next state
    set_robot_speed(DESCENT_SPEED)
    dx = Kp_descent * sensors.correction.dx
    dy = Kp_descent * sensors.correction.dy
    dz = -2.0 # TODO: Change this to use Kp_descent * sensors.correct.dz from height estimate
    controller_logger.info("%d, %.4f, %.4f, %.4f, %.4f", global_state.motion.value, global_state.robot_config.tcp_speed, dx, dy, dz)
    irc5.move_rel_frame(dx, dy, dz, True)

def descend_v2():
    # Main loop will trigger pencil interrupt to go into next state
    dx = Kp_descent * sensors.correction.dx
    dy = Kp_descent * sensors.correction.dy
    dz = -Kp_descent * sensors.correction.dz

    t = max(0, min(1, sensors.correction.dz / 100))
    speed = FIND_DEPTH_SPEED + int((FIND_TARGET_SPEED - FIND_DEPTH_SPEED) * t)
    set_robot_speed(speed)
    controller_logger.info("%d, %.4f, %.4f, %.4f, %.4f", global_state.motion.value, global_state.robot_config.tcp_speed, dx, dy, dz)
    irc5.move_rel_frame(dx, dy, dz)

def record_target():
    if robot_pose_buffer:
        robot_pos = robot_pose_buffer[-1].pos.copy()
        global_state.recorded_points.append(robot_pos)
        irc5.record_target()
        event_logger.info("Calibration point found at:  %.4f,  %.4f,  %.4f", robot_pos[0], robot_pos[1], robot_pos[2])
        time.sleep(1.0)
    else:
        event_logger.error("Unable to store final robot position")

    global_state.motion = MotionState.ASCEND

def find_depth():
    global global_state

    if not pencil_buffer:
        event_logger.warning("No pencil data available for depth finding.")
        irc5.stop_robot()
        return

    latest_pencil = pencil_buffer[-1]
    error = latest_pencil.distance - Z_TARGET_DEPTH

    if abs(error) < Z_TARGET_ACC:
        print(f"Pencil Depth Target Reached: {latest_pencil.distance:.4f} mm")
        event_logger.info("Pencil Depth Target Reached: %.4f mm", latest_pencil.distance)
        record_target()

    dz = error * Kp_pencil
    if abs(latest_pencil.distance - dz) < Z_THRESH: # Make sure to never depress too far and break pencil
        irc5.move_rel_frame(0, 0, dz)
        controller_logger.info("%d, %.4f, %.4f, %.4f, %.4f", global_state.motion.value, global_state.robot_config.tcp_speed, 0, 0, dz)

def ascend():
    global global_state, conveyor_state

    if not robot_pose_buffer:
        event_logger.warning("No robot data available")
        return
    
    ascent_diff = global_state.robot_config.initial_pos[2] - robot_pose_buffer[-1].pos[2]

    if abs(ascent_diff) < ASCENT_HEIGHT_DIFF:
        print("Ascent Complete")
        event_logger.info("Ascent Complete")
        correction = correction_buffer[-1]
        global_state.robot_config.init_est_xy = math.sqrt(correction.dx * correction.dx + correction.dy * correction.dy)
        global_state.robot_config.init_est_z = correction.dz

        if global_state.calibration.value == CalibrationMode.FOUR_POINT.value:
            if len(global_state.recorded_points) >= 4:
                print("Four Point Calibration Complete")
                print(global_state.recorded_points)
                event_logger.info("Four Point Calibration Complete")
                event_logger.info(global_state.recorded_points)

                if RUN_MODE == "four":
                    global_state.motion = MotionState.IDLE
                else:
                    global_state.calibration = CalibrationMode.THREE_POINT
                    global_state.set_target(ThreePointState.FIND_X)
                    global_state.motion = MotionState.FIND_TARGET

                    print("FINDING X TARGET")
                    event_logger.info("FINDING X TARGET")
                    time.sleep(1.0)
            else:
                print("Running the Conveyor")
                event_logger.info("Running the Conveyor")
                irc5.run_conveyor()
                conveyor_state.running = True
                conveyor_state.last_time = time.perf_counter()
                global_state.motion = MotionState.FIND_TARGET

        elif global_state.calibration.value == CalibrationMode.THREE_POINT.value:
            global_state.motion = MotionState.FIND_TARGET
            if global_state.three_point == ThreePointState.FIND_X:
                global_state.set_target(ThreePointState.FIND_Y)
                print("FINDING Y TARGET")
                event_logger.info("FINDING Y TARGET")
                time.sleep(1.0)
            else:
                global_state.set_target(ThreePointState.IDLE)
                global_state.motion = MotionState.IDLE
        
        return

    dz = Kp_ascent * ascent_diff
    controller_logger.info("%d, %.4f, %.4f, %.4f, %.4f", global_state.motion.value, global_state.robot_config.tcp_speed, 0, 0, dz)
    irc5.move_rel_frame(0, 0, dz)

def state_machine():
    global state_last_time
    current_time = time.perf_counter()
    time_interval = current_time - state_last_time

    if global_state.motion == MotionState.FIND_TARGET:
        find_target()
    elif global_state.motion == MotionState.DESCEND:
        descend_v2()
    elif global_state.motion == MotionState.FIND_DEPTH and time_interval >= PENCIL_MOVE_RATE:
        find_depth()
        state_last_time = current_time
    elif global_state.motion == MotionState.ASCEND:
        set_robot_speed(ASCENT_SPEED)
        ascend()
    else:
        return

def move_xy_target():
    global global_state
    dx = X_TARGET - irc5.robot_state.pos[0]
    dy = Y_TARGET - irc5.robot_state.pos[1]
    magnitude = (dx**2 + dy**2)**0.5

    if magnitude > 1.0:
        dx_norm = dx / magnitude
        dy_norm = dy / magnitude
        irc5.move_rel_frame(dx_norm, dy_norm, 0)
    else:
        print(f"Center Target Reached: ({irc5.robot_state.pos[0]:.4f}, {irc5.robot_state.pos[1]:.4f})")
        event_logger.info(f"Center Target Reached: ({irc5.robot_state.pos[0]:.4f}, {irc5.robot_state.pos[1]:.4f})")
        global_state.motion = MotionState.DESCEND
        return

def move_xyz_target():
    global global_state
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
        event_logger.info(f"Final Target Reached: ({irc5.robot_state.pos[0]:.4f}, {irc5.robot_state.pos[1]:.4f}, {irc5.robot_state.pos[2]:.4f})")
        global_state.motion = MotionState.ASCEND
        return

    irc5.move_rel_frame(dx_norm, dy_norm, dz_norm)

def find_init_tags(): 
    global global_state
    if not correction_buffer:
        irc5.run_conveyor()
        conveyor_state.running = True
    else:
        irc5.stop_conveyor()
        conveyor_state.running = False
        global_state.motion = MotionState.FIND_TARGET
        event_logger.info("Tags Found")
        global_state.robot_config.init_est_z = correction_buffer[-1].dz
        return

if __name__ == "__main__":
    RUN_MODE = sys.argv[1] if len(sys.argv) > 1 else "full"
    if RUN_MODE not in ("three", "four", "full"):
        print(f"Unknown mode '{RUN_MODE}', expected one of: three, four, full")
        exit(1)

    global_state.motion = MotionState.FIND_INIT_TAGS
    if RUN_MODE == "three":
        global_state.calibration = CalibrationMode.THREE_POINT
        global_state.set_target(ThreePointState.FIND_CENTER)
        pi_mode = "three-point"
    else:
        global_state.calibration = CalibrationMode.FOUR_POINT
        global_state.set_target(ThreePointState.FIND_CENTER)
        pi_mode = "four-point" if RUN_MODE == "four" else "three-point"

    import robot.sensors as sensors
    print(f"Connecting to sensors... (mode={RUN_MODE})")
    event_logger.info("Connecting to sensors... (mode=%s)", RUN_MODE)
    sensors.connect_sensors()
    sensors.start_sensors()
    sensors.open_sensors(pi_mode)
    print("Connecting to robot...")
    event_logger.info("Connecting to robot...")
    irc5.connect_robot()
    irc5.start_reading_robot()
    time.sleep(2)

    sensors_status = sensors.connection_status()
    robot_status = irc5.connection_status()
    if not sensors_status or not robot_status:
        print(f"Failed to connect to sensors or robot: {sensors_status}, {robot_status}")
        event_logger.error("Failed to connect to sensors or robot: %s, %s", sensors_status, robot_status)
        exit(1)
    else:
        event_logger.info("Successful Connections")
        print("Successful Connections")

    run_start_time = time.perf_counter()
    global_state.motion = MotionState.FIND_INIT_TAGS
    try:
        while True:
            if global_state.motion == MotionState.IDLE:
                break

            sensors_status = sensors.connection_status()
            robot_status = irc5.connection_status()
            if not sensors_status or not robot_status:
                print("Lost connection to sensors or robot.")
                event_logger.error("Lost connection to sensors or robot: %s, %s", sensors_status, robot_status)
                break
            
            if global_state.motion == MotionState.FIND_INIT_TAGS:
                find_init_tags()
            
            if not sensors.correction_buffer and global_state.motion != MotionState.FIND_INIT_TAGS:
                event_logger.warning("No correction data available yet.")
                continue

            if not robot_pose_buffer:
                event_logger.warning("No robot pose data available yet.")
                continue
            
            if show:
                with canvas_lock:
                    cv2.imshow("AprilTag Real-Time Map", sensors.canvas)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if pencil_buffer and pencil_buffer[-1].active:
                if global_state.motion.value < MotionState.FIND_DEPTH.value:
                    global_state.motion = MotionState.FIND_DEPTH
                    print("Pencil Detected. Switching to FIND_DEPTH mode.")
                    event_logger.info("Pencil Detected. Switching to FIND_DEPTH mode.")
                    record_target()

            state_machine()

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        elapsed = time.perf_counter() - run_start_time
        print(f"Total elapsed time: {elapsed:.2f} seconds")
        print("Disconnecting from robot...")
        event_logger.info(f"Total elapsed time: {elapsed:.2f} seconds")
        event_logger.info("Stopping the conveyor, Disconnecting the robot & sensors...")
        irc5.stop_conveyor()
        irc5.stop_robot()
        irc5.stop_reading_robot()
        irc5.disconnect_robot()
        sensors.stop_sensors()