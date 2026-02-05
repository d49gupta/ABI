import socket
import numpy as np
from dataclasses import dataclass
from scripts.logger import CSVLogger

# --- DATACLASS ---
@dataclass
class RobotConfig:
    ip_address: str = '127.0.0.1'
    port: int = 5000
    socket = None
    timeout: float = 5.0

class robotState:
    initial_pos : np.ndarray = None
    pos : np.ndarray = None
    orientation : np.ndarray = None

# --- GLOBALS ---
robot_logger = CSVLogger(name="robot", log_dir="test_logs")

# --- STATE ---
robot_state = robotState()
robot_config = RobotConfig()

def connect_robot():
    robot_config.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    robot_config.socket.connect((robot_config.ip_address, robot_config.port))
    robot_config.socket.settimeout(robot_config.timeout)

def read_robot_state():
    try:
        data = robot_config.socket.recv(1024).decode('utf-8')
        robot_values = [float(val) for val in data.split(',')]
        robot_state.pos = np.array(robot_values[0:3])
        robot_state.orientation = np.array(robot_values[3:7])

        if robot_state.initial_pos is None:
            robot_state.initial_pos = robot_state.pos.copy()

        robot_logger.info("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", robot_state.pos[0], robot_state.pos[1], robot_state.pos[2], 
                        robot_state.orientation[0], robot_state.orientation[1], robot_state.orientation[2], robot_state.orientation[3])
    except socket.timeout:
        print("Timeout: No data received from robot.")

def send_cartesian_command(dx, dy, dz):
    command = f"1, {dx}, {dy}, {dz}"
    robot_config.socket.sendall(command.encode('utf-8'))

def disconnect_robot():
    robot_config.socket.close()