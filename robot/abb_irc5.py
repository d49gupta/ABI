import socket
import numpy as np
from robot.globals import *

# --- STATES ---
robot_state = robotState()

def connect_robot():
    try:
        robot_config.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        robot_config.socket.connect((robot_config.ip_address, robot_config.port))
        robot_config.socket.settimeout(robot_config.timeout)
        robot_config.connected = True
        robot_config.robot_file = robot_config.socket.makefile('r')
    except (socket.timeout, ConnectionRefusedError, OSError) as e:
        print(f"Connection failed: {e}")
        robot_config.connected = False

def connection_status():
    return robot_config.connected

def read_robot_state():
    try:
        line = robot_config.robot_file.readline()
        # data = robot_config.socket.recv(1024).decode('utf-8')
        if line:
            # robot_values = [float(val) for val in data.split(',')]
            robot_values = [float(val) for val in line.strip().split(',')]
            robot_state.pos = np.array(robot_values[0:3])
            robot_state.orientation = np.array(robot_values[3:7])

        if robot_state.initial_pos is None:
            robot_state.initial_pos = robot_state.pos.copy()

        robot_logger.info("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", robot_state.pos[0], robot_state.pos[1], robot_state.pos[2], 
                        robot_state.orientation[0], robot_state.orientation[1], robot_state.orientation[2], robot_state.orientation[3])
    except socket.timeout:
        print("Timeout: No data received from robot.")

def get_displacement():
    if robot_state.initial_pos is None or robot_state.pos is None:
        return np.zeros(3)
    return robot_state.pos - robot_state.initial_pos

def move_rel_frame(dx, dy, dz):
    command = f"1, {dx}, {dy}, {dz}"
    robot_config.socket.sendall(command.encode('utf-8'))

def stop_robot():
    command = f"2"
    robot_config.socket.sendall(command.encode('utf-8'))

def move_robot_frame(x, y, z):
    command = f"3, {x}, {y}, {z}"
    robot_config.socket.sendall(command.encode('utf-8'))

def disconnect_robot():
    robot_config.socket.close()