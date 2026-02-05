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

class robotState:
    initial_pos : np.ndarray = None
    pos : np.ndarray = None

# --- GLOBALS ---
robot_logger = CSVLogger(name="robot", log_dir="test_logs")

# --- STATE ---
robot_state = robotState()
robot_config = RobotConfig()

def connect_robot():
    robot_config.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    robot_config.socket.connect((robot_config.ip_address, robot_config.port))

def read_robot_state():
    data = robot_config.socket.recv(1024).decode('utf-8')
    pos_values = [float(val) for val in data.split(',')]
    robot_state.pos = np.array(pos_values)

def send_cartesian_command(dx, dy, dz):
    command = f"X:{dx}"
    robot_config.socket.sendall(command.encode('utf-8'))

def disconnect_robot():
    robot_config.socket.close()