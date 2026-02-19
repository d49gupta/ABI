import robot.abb_irc5 as irc5
import robot.sensors as sensors
from robot.globals import *
import time

def move_xy_sensors():
    global motion_state
    if not sensors.correction_buffer:
        controller_logger.warning("Not enough correction data for camera smoothing.")
        return
    
    global smooth_dx, smooth_dy
    camera_curr_correction = sensors.correction_buffer[-1]
    smooth_dx = (alpha_camera * camera_curr_correction.dx) + (1 - alpha_camera) * smooth_dx
    smooth_dy = (alpha_camera * camera_curr_correction.dy) + (1 - alpha_camera) * smooth_dy
    magnitude = (smooth_dx**2 + smooth_dy**2)**0.5

    if not irc5.robot_pose_buffer:
        controller_logger.warning("Not enough robot pose data for camera smoothing.")
        return

    if magnitude > XY_TARGET_ACC: 
        robot_pose = irc5.robot_pose_buffer[-1].pos
        dx = robot_pose[0] + Kp_camera * smooth_dx
        dy = robot_pose[1] - Kp_camera * smooth_dy
        controller_logger.info("%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f", smooth_dx, smooth_dy, 
                         camera_curr_correction.dx, camera_curr_correction.dy, robot_pose[0], robot_pose[1], dx, dy)
        # irc5.move_robot_frame(dx, dy, 0) # Larger corrections
    else:
        print(f"Camera Correction Target Reached")
        controller_logger.info("Camera Correction Target Reached (%.4f, %.4f)", robot_pose[0], robot_pose[1])
        motion_state = MotionState.DESCEND

def move_xyz_sensors():
    # dx and dy magnitude should be less than 1.0
    # keep moving down by dz = -1.0 to allow for small xy corrections
    # Main loop will trigger pencil interrupt to go into next state
    dx = Kp_camera * sensors.correction.dx
    dy = Kp_camera * sensors.correction.dy
    dz = -1.0
    
    controller_logger.info("%.4f, %.4f, %.4f", dx, dy, dz)
    # irc5.move_rel_frame(dx, dy, dz)

def find_pencil_depth():
    global motion_state, final_robot_pose

    if not pencil_buffer:
        controller_logger.warning("No pencil data available for depth finding.")
        irc5.stop_robot()
        return

    latest_pencil = pencil_buffer[-1]
    error = latest_pencil.distance - Z_TARGET

    if abs(error) < Z_TARGET_ACC:
        print(f"Pencil Depth Target Reached: {latest_pencil.distance:.4f} mm")
        controller_logger.info("Pencil Depth Target Reached: %.4f mm", latest_pencil.distance)
        final_robot_pose = irc5.robot_state.pos.copy()
        time.sleep(5.0)
        motion_state = MotionState.ASCEND
        return

    dz = error * Kp_pencil
    if abs(latest_pencil.distance - dz) < Z_THRESH: # Make sure to never depress too far and break pencil
        irc5.move_rel_frame(0, 0, dz)
        controller_logger.info("%.4f, %.4f, %.4f", 0, 0, dz)

def ascent():
    global motion_state
    ascent_diff = irc5.robot_state.initial_pos[2] - irc5.robot_state.pos[2]
    if abs(ascent_diff) < 5.0:
        print("Ascent Complete")
        controller_logger.info("Ascent Complete")
        motion_state = MotionState.IDLE

    dz = ascent_diff * Kp_ascent
    controller_logger.info("%.4f, %.4f, %.4f", 0, 0, dz)
    irc5.move_rel_frame(0, 0, dz)

def move_xy_target():
    global motion_state
    dx = X_TARGET - irc5.robot_state.pos[0]
    dy = Y_TARGET - irc5.robot_state.pos[1]
    magnitude = (dx**2 + dy**2)**0.5

    if magnitude > 1.0:
        dx_norm = dx / magnitude
        dy_norm = dy / magnitude
        irc5.robot_logger.info("%.4f, %.4f, %.4f", dx, dy, 0)
        irc5.move_rel_frame(dx_norm, dy_norm, 0)
    else:
        print(f"Center Target Reached: ({irc5.robot_state.pos[0]:.4f}, {irc5.robot_state.pos[1]:.4f})")
        irc5.robot_logger.info("Center Target Reached (%.4f, %.4f)", irc5.robot_state.pos[0], irc5.robot_state.pos[1])
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
        irc5.robot_logger.info("Final Target Reached: (%.4f, %.4f, %.4f)", irc5.robot_state.pos[0], irc5.robot_state.pos[1], irc5.robot_state.pos[2])
        final_robot_pose = irc5.robot_state.pos.copy()
        motion_state = MotionState.ASCEND
        return

    irc5.robot_logger.info("%.4f, %.4f, %.4f", dx, dy, dz)
    irc5.move_rel_frame(dx_norm, dy_norm, dz_norm)

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
        # TODO: Send command to pi to stop vision processing