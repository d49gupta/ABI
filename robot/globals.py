from scripts.logger import CSVLogger
from dataclasses import dataclass
import numpy as np
from enum import Enum
from collections import deque
import threading
import time

# --- ENUMS ---
class MotionState(Enum):
    IDLE = 0
    FIND_CENTER = 1
    DESCEND = 2
    FIND_DEPTH = 3
    ASCEND = 4

# --- DATACLASSES ---
@dataclass
class pencilState:
    raw: int = 0
    distance: float = 0.0
    active: bool = False
    timestamp: float = 0

@dataclass
class cameraState:
    center_x: int = 0
    center_y: int = 0
    scale: float = 0.0
    timestamp: float = 0

@dataclass
class MQTTState:
    mqtt_broker: str = "127.0.0.1"
    camera_topic: str = "camera/detections"
    pencil_topic: str = "pencil/reading"
    port: int = 1883
    client = None
    msg_count: int = 0
    start_time: float = 0

@dataclass
class CorrectionState:
    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0
    timestamp: float = 0

@dataclass
class RobotConfig:
    ip_address: str = '127.0.0.1'
    port: int = 5000
    socket = None
    timeout: float = 5.0 # TODO: adjust timeout as needed, maybe make it non-blocking with select instead
    connected: bool = False
    msg_count: int = 0
    robot_file = None
    read_thread = None
    stop_trigger = None

@dataclass
class robotState:
    initial_pos : np.ndarray = None
    pos : np.ndarray = None
    orientation : np.ndarray = None
    timestamp: int = 0

# --- GLOBALS ---
# MQTT_BROKER = "fe80::80ee:98fe:7fcb:95c3%16"
SIM_MQTT_BROKER = "127.0.0.1"
MQTT_BROKER = "172.20.10.5"
ROBOT_SIM_IP = "127.0.0.1"
ROBOT_REAL_IP = "10.60.70.51"
WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480
img_center_x = WINDOW_WIDTH // 2
img_center_y = WINDOW_HEIGHT // 2

X_TARGET = 600
Y_TARGET = 18.93
Z_TARGET = -905.67
Z_THRESH = 8.0
Z_TARGET = 4.0
Z_ACTIVE = 0.25
XY_TARGET_ACC = 1.0
Z_TARGET_ACC = 1.0
PENCIL_Z_OFFSET = 40 # mm (165)
ROBOT_PUBLISH_RATE = 0.1 # seconds, should not be faster than camera frequency
PENCIL_MOVE_RATE = 1.0

canvas = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH, 3), dtype=np.uint8)
canvas_lock = threading.Lock()
show = True
show_camera_info = False

# --- BUFFERS ---
pencil_buffer = deque(maxlen=50)
camera_buffer = deque(maxlen=10)
correction_buffer = deque(maxlen=10)
robot_pose_buffer = deque(maxlen=25)

# --- STATES ---
correction = CorrectionState()
pencil_sample = pencilState()
camera_sample = cameraState()
robot_state = robotState()
motion_state = MotionState.IDLE
final_robot_pose = None
state_last_time = time.perf_counter()

# --- LOGGERS ---
camera_logger = CSVLogger(name="camera", log_dir="test_logs")
pencil_logger = CSVLogger(name="pencil", log_dir="test_logs")
robot_logger = CSVLogger(name="robot", log_dir="test_logs")
correction_logger = CSVLogger(name="diff", log_dir="test_logs")
camera_perf_logger = CSVLogger(name="camera_perf", log_dir="test_logs")
controller_logger = CSVLogger(name="controller", log_dir="test_logs")

# --- CONFIGS ---
subscriber = MQTTState(mqtt_broker=MQTT_BROKER)
robot_config = RobotConfig(ip_address=ROBOT_SIM_IP)

# --- CONTROLLERS ---
alpha_camera = 0.1
smooth_dx = 0.0
smooth_dy = 0.0
Kp_camera = 0.1
Kp_pencil = 0.1
Kp_ascent = 0.1