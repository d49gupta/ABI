import socket
import struct
import numpy as np
from robot.globals import *
from dataclasses import replace
import time

POSE_MSG_LEN = 20   # 5 x float32: x, y, z, conveyor_axis, reported_speed
CMD_MSG_LEN = 16    # 4 x float32: cmd_id, f1, f2, f3
STRUCT_ENDIAN = '<' # must match RAPID's native PackRawBytes/UnpackRawBytes byte order;
                     # flip to '>' here if values come back garbled/huge/NaN

def connect_robot():
    try:
        global_state.robot_config.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        global_state.robot_config.socket.settimeout(global_state.robot_config.timeout)
        global_state.robot_config.socket.connect((global_state.robot_config.ip_address, global_state.robot_config.port))
        global_state.robot_config.connected = True
        global_state.robot_config.read_thread = threading.Thread(target=read_robot_state, daemon=True)
        global_state.robot_config.stop_trigger = threading.Event()
        global_state.robot_config.last_time = time.perf_counter()

    except (socket.timeout, ConnectionRefusedError, OSError) as e:
        print(f"Connection failed: {e}")
        global_state.robot_config.connected = False

def connection_status():
    return global_state.robot_config.connected and global_state.robot_config.msg_count > 0

def _recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Socket closed by robot")
        buf += chunk
    return buf

def read_robot_state():
    while not global_state.robot_config.stop_trigger.is_set():
        try:
            data = _recv_exact(global_state.robot_config.socket, POSE_MSG_LEN)
            x, y, z, conveyor_axis, reported_speed = struct.unpack(STRUCT_ENDIAN + '5f', data)

            global_state.robot_config.msg_count += 1
            robot_state.pos = np.array([x, y, z])
            robot_state.conveyor_axis = conveyor_axis
            robot_state.reported_speed = reported_speed

            if global_state.robot_config.initial_pos is None:
                global_state.robot_config.initial_pos = robot_state.pos.copy()

            curr_robot_state = replace(robot_state)
            robot_pose_buffer.append(curr_robot_state)
            robot_logger.info("%d, %.2f, %.2f, %.2f, %.2f, %.2f", global_state.motion.value, curr_robot_state.pos[0], curr_robot_state.pos[1], curr_robot_state.pos[2],
                            curr_robot_state.conveyor_axis, curr_robot_state.reported_speed)
        except socket.timeout:
            robot_logger.warning("Timeout: No data received from robot.")
            print("Timeout: No data received from robot.")
        except ConnectionError as e:
            robot_logger.error("Connection error: %s", e)
            break

def start_reading_robot():
    global_state.robot_config.read_thread.start()

def stop_reading_robot():
    global_state.robot_config.stop_trigger.set()
    global_state.robot_config.read_thread.join(timeout=2.0)

def get_displacement():
    if global_state.robot_config.initial_pos is None or robot_state.pos is None:
        return np.zeros(3)
    return robot_state.pos - global_state.robot_config.initial_pos

def _send_command(cmd_id: float, f1: float = 0.0, f2: float = 0.0, f3: float = 0.0):
    packed = struct.pack(STRUCT_ENDIAN + '4f', cmd_id, f1, f2, f3)
    global_state.robot_config.socket.sendall(packed)

def move_rel_frame(dx, dy, dz, override_cmd=False):
    global global_state
    current_time = time.perf_counter()
    if current_time - global_state.robot_config.last_time < ROBOT_PUBLISH_RATE:
        return

    # Don't get ahead of the robot: wait for RAPID to report a fresh pose
    # (i.e. finish its last Send/Receive/Move cycle) before sending the next command.
    if global_state.robot_config.msg_count == global_state.robot_config.last_acked_msg_count:
        return

    # only send if the requested correction itself is significant enough to bother moving
    magnitude = (dx**2 + dy**2 + dz**2) ** 0.5
    if magnitude < MOVE_DEADBAND_MM and not override_cmd:
        return

    global_state.robot_config.last_time = current_time
    global_state.robot_config.last_acked_msg_count = global_state.robot_config.msg_count

    _send_command(1, -dx, -dy, dz)

def stop_robot():
    _send_command(2)

def set_speed(v_tcp):
    _send_command(6, v_tcp)

def move_robot_frame(x, y, z):
    _send_command(3, -x, -y, z)

def yaw_robot(angle):
    _send_command(4, angle)

def run_conveyor():
    _send_command(8)

def stop_conveyor():
    _send_command(9)

def record_target():
    _send_command(7)

def move_robot_home():
    _send_command(5)

def disconnect_robot():
    global_state.robot_config.socket.close()

if __name__ == "__main__":
    connect_robot()
    start_reading_robot()
    time.sleep(2)  # Wait for connection to establish
    run_conveyor()

    try:
        while True:
            move_rel_frame(10, 0, 0, True)
            print(f"Current Position: {robot_state.pos}, Conveyor Axis: {robot_state.conveyor_axis}, Speed: {robot_state.reported_speed}")
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        print("Disconnecting from robot...")
        stop_conveyor()
        move_robot_home()
        stop_reading_robot()
        disconnect_robot()
