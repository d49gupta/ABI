import socket
import numpy as np
from robot.globals import *
from dataclasses import replace
import time

def connect_robot():
    try:
        global_state.robot_config.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        global_state.robot_config.socket.connect((global_state.robot_config.ip_address, global_state.robot_config.port))
        global_state.robot_config.socket.settimeout(global_state.robot_config.timeout)
        global_state.robot_config.connected = True
        global_state.robot_config.robot_file = global_state.robot_config.socket.makefile('r')
        global_state.robot_config.read_thread = threading.Thread(target=read_robot_state, daemon=True)
        global_state.robot_config.stop_trigger = threading.Event()
        global_state.robot_config.last_time = time.perf_counter()

    except (socket.timeout, ConnectionRefusedError, OSError) as e:
        print(f"Connection failed: {e}")
        global_state.robot_config.connected = False

def connection_status():
    return global_state.robot_config.connected and global_state.robot_config.msg_count > 0

def read_robot_state():
    while not global_state.robot_config.stop_trigger.is_set():
        try:
            line = global_state.robot_config.robot_file.readline()
            # line = global_state.robot_config.socket.recv(1024).decode('utf-8')
            if line:
                global_state.robot_config.msg_count += 1
                # robot_values = [float(val) for val in line.split(',')]
                robot_values = [float(val) for val in line.strip().split(',')]
                robot_state.pos = np.array(robot_values[0:3])
                robot_state.orientation = np.array(robot_values[3:7])
                robot_state.conveyor_axis = robot_values[7]

                if global_state.robot_config.initial_pos is None:
                    global_state.robot_config.initial_pos = robot_state.pos.copy()

                curr_robot_state = replace(robot_state)
                robot_pose_buffer.append(curr_robot_state)
                robot_logger.info("%d, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %d", global_state.motion.value, curr_robot_state.pos[0], curr_robot_state.pos[1], curr_robot_state.pos[2], 
                                curr_robot_state.orientation[0], curr_robot_state.orientation[1], curr_robot_state.orientation[2], curr_robot_state.orientation[3],
                                curr_robot_state.conveyor_axis)
        except socket.timeout:
            robot_logger.warning("Timeout: No data received from robot.")
            print("Timeout: No data received from robot.")

def start_reading_robot():
    global_state.robot_config.read_thread.start()

def stop_reading_robot():
    global_state.robot_config.stop_trigger.set()
    global_state.robot_config.read_thread.join(timeout=2.0)

def get_displacement():
    if global_state.robot_config.initial_pos is None or robot_state.pos is None:
        return np.zeros(3)
    return robot_state.pos - global_state.robot_config.initial_pos

def move_rel_frame(dx, dy, dz):
    global global_state
    current_time = time.perf_counter()
    if current_time - global_state.robot_config.last_time < ROBOT_PUBLISH_RATE:
        return
    
    global_state.robot_config.last_time = current_time
    command = f"1, {dx}, {dy}, {dz}"
    global_state.robot_config.socket.sendall(command.encode('utf-8'))

def stop_robot():
    command = f"2"
    global_state.robot_config.socket.sendall(command.encode('utf-8'))

def move_robot_frame(x, y, z):
    command = f"3, {x}, {y}, {z}"
    global_state.robot_config.socket.sendall(command.encode('utf-8'))

def yaw_robot(angle):
    command = f"4,{angle}"
    global_state.robot_config.socket.sendall(command.encode('utf-8'))

def run_conveyor():
    command = f"8"
    global_state.robot_config.socket.sendall(command.encode('utf-8'))

def stop_conveyor():
    command = f"9"
    global_state.robot_config.socket.sendall(command.encode('utf-8'))

def record_target():
    command = f"7"
    global_state.robot_config.socket.sendall(command.encode('utf-8'))

def move_robot_home():
    command = f"5"
    global_state.robot_config.socket.sendall(command.encode('utf-8'))

def disconnect_robot():
    global_state.robot_config.socket.close()

if __name__ == "__main__":
    connect_robot()
    start_reading_robot()
    time.sleep(2)  # Wait for connection to establish

    try:
        run_conveyor()
        while True:
            move_rel_frame(10, 0, 0)
            print(f"Current Position: {robot_state.pos}, Orientation: {robot_state.orientation}")
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        print("Disconnecting from robot...")
        stop_conveyor()
        stop_reading_robot()
        disconnect_robot()
