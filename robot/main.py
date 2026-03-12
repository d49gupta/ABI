import robot.abb_irc5 as irc5
import robot.sensors as sensors
from robot.globals import *
import time
import cv2
import numpy as np

motion_state = MotionState.IDLE
calibration_mode = CalibrationMode.FOUR_POINT
center_robot_pose = None  # Saved robot pose above tag 4 center

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
            # Branch on calibration mode to store in correct results list
            if calibration_mode == CalibrationMode.FOUR_POINT:
                four_point_pos.append(robot_pose_buffer[-1].pos.copy())
            elif calibration_mode == CalibrationMode.THREE_POINT:
                three_point_pos.append(robot_pose_buffer[-1].pos.copy())
                print(f"3-point position {len(three_point_pos)} recorded.")
            irc5.record_target()
        else:
            controller_logger.error("Unable to store final robot position")

        motion_state = MotionState.ASCEND
        return

    dz = error * Kp_pencil
    if abs(latest_pencil.distance - dz) < Z_THRESH:  # Never depress too far and break pencil
        irc5.move_rel_frame(0, 0, dz)
        controller_logger.info("%d, %.4f, %.4f, %.4f", motion_state.value, 0, 0, dz)

def ascent():
    global motion_state, conveyor_state

    ascent_diff = irc5.robot_state.initial_pos[2] - irc5.robot_state.pos[2]
    if abs(ascent_diff) < ASCENT_HEIGHT_DIFF:
        print("Ascent Complete")
        controller_logger.info("Ascent Complete")

        if calibration_mode == CalibrationMode.FOUR_POINT:
            if len(four_point_pos) >= 4:
                print("Four Point Calibration Complete")
                motion_state = MotionState.IDLE
            else:
                # More 4-point positions needed, run conveyor to next point
                print("Running the Conveyor")
                conveyor_state.running = True
                irc5.run_conveyor()
                conveyor_state.last_time = time.perf_counter()
                motion_state = MotionState.FIND_CENTER

        elif calibration_mode == CalibrationMode.THREE_POINT:
            if len(three_point_pos) == 0:
                # Ascended from center — record pose and move to tag 5
                record_center_and_move_to_tag5()
            elif len(three_point_pos) == 1:
                # Ascended from tag 5 — return to center then go to tag 6
                motion_state = MotionState.RETURN_TO_CENTER
            elif len(three_point_pos) >= 2:
                # Ascended from tag 6 — all 3 points recorded
                print("Three Point Calibration Complete")
                motion_state = MotionState.IDLE
        return  # Do not move if ascent is complete

    # Still ascending
    irc5.move_rel_frame(0, 0, 2.0)
    controller_logger.info("%d, 0, 0, 2.0", motion_state.value)

def record_center_and_move_to_tag5():
    """
    Records the current robot pose as the center reference point above
    tag 4, then transitions to moving toward the predicted center of tag 5.
    """
    global motion_state, center_robot_pose

    if robot_pose_buffer:
        center_robot_pose = robot_pose_buffer[-1].pos.copy()
        print(f"Center pose recorded: {center_robot_pose}")
        controller_logger.info("Center pose recorded: %.2f, %.2f, %.2f",
                               center_robot_pose[0], center_robot_pose[1], center_robot_pose[2])
        motion_state = MotionState.MOVE_TO_TAG5
    else:
        controller_logger.error("No robot pose available to record center.")
        motion_state = MotionState.IDLE

def return_to_center():
    """
    Commands the robot toward the saved center pose above tag 4 using
    absolute positioning. Polls position each loop iteration and transitions
    to MOVE_TO_TAG6 once within 5mm of the saved pose.
    """
    global motion_state

    if center_robot_pose is None:
        controller_logger.error("No center pose saved to return to.")
        motion_state = MotionState.IDLE
        return

    irc5.move_to_saved_pose(center_robot_pose)

    delta = np.linalg.norm(center_robot_pose - irc5.robot_state.pos)
    if delta < 5.0:
        print("Returned to center. Moving to tag 6.")
        controller_logger.info("Returned to center pose.")
        motion_state = MotionState.MOVE_TO_TAG6

def move_to_tag_target(tag_target_pixel, tag_scale, threshold=1.0):
    """
    Moves the robot XY toward a pixel-space target using the current camera
    scale to convert pixel error to mm. Called each loop iteration until
    the target is reached.

    Args:
        tag_target_pixel (tuple): (center_x, center_y) predicted pixel target.
        tag_scale (float): Current mm/pixel scale from the tag.
        threshold (float): Pixel error in px considered close enough to stop.

    Returns:
        bool: True if target reached, False if still moving.
    """
    dx_px = tag_target_pixel[0] - img_center_x
    dy_px = tag_target_pixel[1] - img_center_y
    magnitude = (dx_px**2 + dy_px**2)**0.5

    if magnitude < threshold:
        return True

    dx_mm = dx_px * tag_scale * Kp_camera
    dy_mm = dy_px * tag_scale * Kp_camera
    irc5.move_rel_frame(dx_mm, dy_mm, 0.0)
    return False

def state_machine():
    global state_last_time, motion_state
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
    elif motion_state == MotionState.RETURN_TO_CENTER:
        return_to_center()
    elif motion_state == MotionState.MOVE_TO_TAG5:
        if sensors.tag5_target and sensors.tag5_scale:
            reached = move_to_tag_target(sensors.tag5_target, sensors.tag5_scale)
            if reached:
                print("Tag 5 target reached. Descending.")
                controller_logger.info("Tag 5 target reached.")
                motion_state = MotionState.DESCEND
        else:
            controller_logger.warning("No tag 5 target available yet.")
    elif motion_state == MotionState.MOVE_TO_TAG6:
        if sensors.tag6_target and sensors.tag6_scale:
            reached = move_to_tag_target(sensors.tag6_target, sensors.tag6_scale)
            if reached:
                print("Tag 6 target reached. Descending.")
                controller_logger.info("Tag 6 target reached.")
                motion_state = MotionState.DESCEND
        else:
            controller_logger.warning("No tag 6 target available yet.")
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


# States where pencil interrupt should NOT trigger a depth search
PENCIL_INTERRUPT_BLOCKED_STATES = {
    MotionState.MOVE_TO_TAG5,
    MotionState.MOVE_TO_TAG6,
    MotionState.RETURN_TO_CENTER,
    MotionState.ASCEND,
    MotionState.IDLE,
}

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
    motion_state = MotionState.FIND_CENTER
    try:
        while True:
            if motion_state == MotionState.IDLE:
                break

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

            # Pencil interrupt — only trigger in valid states
            if (motion_state not in PENCIL_INTERRUPT_BLOCKED_STATES
                    and len(pencil_buffer) >= 3
                    and pencil_buffer[-1].active
                    and pencil_buffer[-2].active
                    and pencil_buffer[-3].active):
                if motion_state != MotionState.FIND_DEPTH:
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