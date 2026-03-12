import robot.abb_irc5 as irc5
import robot.sensors as sensors
from robot.globals import *
import time
import cv2

motion_state = MotionState.IDLE
calibration_mode = CalibrationMode.FOUR_POINT

def get_motion_state():
    return motion_state.value

def move_xy_sensors():
    global motion_state, conveyor_state

    if not sensors.correction_buffer:
        controller_logger.warning("Not enough correction data for camera smoothing.")
        return
    
    global smooth_dx, smooth_dy
    camera_curr_correction = sensors.correction_buffer[-1]
    smooth_dx = (alpha_camera * camera_curr_correction.dx) + (1 - alpha_camera) * smooth_dx
    smooth_dy = (alpha_camera * camera_curr_correction.dy) + (1 - alpha_camera) * smooth_dy
    magnitude = (smooth_dx**2 + smooth_dy**2)**0.5

    if conveyor_state.running and conveyor_state.last_time:
        irc5.run_conveyor()
        curr_time = time.perf_counter()
        if curr_time - conveyor_state.last_time >= CONVEYOR_MOVE_TIME:
            print("Stopping the conveyor")    
            conveyor_state.running = False
            conveyor_state.last_time = None

    if not conveyor_state.running:
            irc5.stop_conveyor()
            # conveyor_state.last_time = None        

    if magnitude > XY_TARGET_ACC or conveyor_state.running:
        dx = Kp_camera * smooth_dx
        dy = Kp_camera * smooth_dy
        controller_logger.info("%d, %.4f, %.4f, %.4f", motion_state.value, dx, dy, 0.0)
        irc5.move_rel_frame(dx, dy, 0.0)
    else:
        print(f"Camera Correction Target Reached")
        controller_logger.info("Camera Correction Target Reached")
        smooth_dx = 0
        smooth_dy = 0
        motion_state = MotionState.DESCEND

def move_xyz_sensors():
    # Main loop will trigger pencil interrupt to go into next state
    dx = Kp_camera * sensors.correction.dx
    dy = Kp_camera * sensors.correction.dy
    dz = -2.0
    controller_logger.info("%d, %.4f, %.4f, %.4f", motion_state.value, dx, dy, dz)
    irc5.move_rel_frame(dx, dy, dz)

def find_pencil_depth():
    global motion_state

    if not pencil_buffer:
        controller_logger.warning("No pencil data available for depth finding.")
        irc5.stop_robot()
        return

    latest_pencil = pencil_buffer[-1]
    error = latest_pencil.distance - Z_TARGET_DEPTH

    if abs(error) < Z_TARGET_ACC:
        print(f"Pencil Depth Target Reached: {latest_pencil.distance:.4f} mm")
        controller_logger.info("Pencil Depth Target Reached: %.4f mm", latest_pencil.distance)
        
        if robot_pose_buffer:
            four_point_pos.append(robot_pose_buffer[-1].pos.copy())
            irc5.record_target()
            time.sleep(1.0)
        else:
            controller_logger.error("Unable to store final robot position")

        motion_state = MotionState.ASCEND

    dz = error * Kp_pencil
    if abs(latest_pencil.distance - dz) < Z_THRESH: # Make sure to never depress too far and break pencil
        irc5.move_rel_frame(0, 0, dz)
        controller_logger.info("%d, %.4f, %.4f, %.4f", motion_state.value, 0, 0, dz)

def ascent():
    global motion_state, conveyor_state
    ascent_diff = irc5.robot_state.initial_pos[2] - irc5.robot_state.pos[2]
    if abs(ascent_diff) < ASCENT_HEIGHT_DIFF:
        print("Ascent Complete")
        controller_logger.info("Ascent Complete")

        if calibration_mode.value == CalibrationMode.FOUR_POINT.value and len(four_point_pos) >= 4:
            print("Four Point Calibration Complete")
            motion_state = MotionState.IDLE
            three_point_pos.append(robot_pose_buffer[-1].pos.copy())
            print(four_point_pos)
            print("")
            print(three_point_pos)
        else:
            three_point_pos.append(robot_pose_buffer[-1].pos.copy())
            print("Running the Conveyor")
            conveyor_state.running = True
            irc5.run_conveyor()
            conveyor_state.last_time = time.perf_counter()
            motion_state = MotionState.FIND_CENTER

    dz = 2.0
    controller_logger.info("%d, %.4f, %.4f, %.4f", motion_state.value, 0, 0, dz)
    irc5.move_rel_frame(0, 0, dz)

def state_machine():
    global state_last_time
    current_time = time.perf_counter()
    time_interval = current_time - state_last_time
    if motion_state == MotionState.FIND_CENTER:
        move_xy_sensors()
    elif motion_state == MotionState.DESCEND:
        move_xyz_sensors()
    elif motion_state == MotionState.FIND_DEPTH and time_interval >= PENCIL_MOVE_RATE:
        find_pencil_depth()
        state_last_time = current_time
    elif motion_state == MotionState.ASCEND:
        ascent()
    else:
        return

def move_xy_target():
    global motion_state
    dx = X_TARGET - irc5.robot_state.pos[0]
    dy = Y_TARGET - irc5.robot_state.pos[1]
    magnitude = (dx**2 + dy**2)**0.5

    if magnitude > 1.0:
        dx_norm = dx / magnitude
        dy_norm = dy / magnitude
        irc5.move_rel_frame(dx_norm, dy_norm, 0)
    else:
        print(f"Center Target Reached: ({irc5.robot_state.pos[0]:.4f}, {irc5.robot_state.pos[1]:.4f})")
        motion_state = MotionState.DESCEND
        return

def move_xyz_target():
    global motion_state, final_robot_pose
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
        final_robot_pose = irc5.robot_state.pos.copy()
        motion_state = MotionState.ASCEND
        return

    irc5.move_rel_frame(dx_norm, dy_norm, dz_norm)

def find_init_tags(): 
    global motion_state
    if not camera_buffer:
        irc5.run_conveyor()
    else:
        irc5.stop_conveyor()
        motion_state = MotionState.FIND_CENTER
        controller_logger.info("Tags Found")
        return


if __name__ == "__main__":
    print("Connecting to sensors")
    sensors.connect_sensors()
    sensors.start_sensors()
    print("Connecting to robot...")
    irc5.connect_robot()
    irc5.start_reading_robot()
    time.sleep(2)

    # TODO: Send command to change speed of arm in move_rel

    if not sensors.connection_status() or not irc5.connection_status():
        print(sensors.connection_status(), irc5.connection_status())
        print("Failed to connect to sensors or robot.")
        exit(1)
    else:
        print("Successful Connections")

    last_time = time.perf_counter()
    motion_state = MotionState.FIND_INIT_TAGS
    try:
        while True:
            if motion_state == MotionState.IDLE:
                break
            
            if motion_state == MotionState.FIND_INIT_TAGS:
                find_init_tags()

            if not sensors.connection_status() or not irc5.connection_status():
                print("Lost connection to sensors or robot.")
                break
            
            if not sensors.correction_buffer:
                controller_logger.warning("No correction data available yet.")
                continue

            if not irc5.robot_pose_buffer:
                controller_logger.warning("No robot pose data available yet.")
                continue
            
            if show:
                with canvas_lock:
                    cv2.imshow("AprilTag Real-Time Map", sensors.canvas)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if pencil_buffer and pencil_buffer[-1].active and pencil_buffer[-2].active and pencil_buffer[-3].active:
                if motion_state.value < MotionState.FIND_DEPTH.value:
                    motion_state = MotionState.FIND_DEPTH
                    controller_logger.info("Pencil Detected. Switching to FIND_DEPTH mode.")
                    print("Pencil Detected. Switching to FIND_DEPTH mode.")

            state_machine()

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        print("Disconnecting from robot...")
        irc5.stop_robot()
        irc5.stop_reading_robot()
        irc5.disconnect_robot()
        sensors.stop_sensors()
        # TODO: Send command to pi to stop vision processing