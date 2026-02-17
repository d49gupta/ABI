from scripts.logger import CSVLogger
from dataclasses import dataclass
import numpy as np
from enum import Enum

# --- ENUMS ---
class MotionState(Enum):
    IDLE = 0
    FIND_CENTER = 1
    DESCEND = 2
    FIND_DEPTH = 3
    ASCEND = 4

# --- DATACLASSES ---
@dataclass
class PencilReading:
    raw: int = 0
    millimeters: float = 0.0
    flag: int = 0
    active: bool = False

@dataclass
class CameraDetection:
    center_x: int = 0
    center_y: int = 0
    scale: float = 0.0
    # TODO: Add timestamp later for timeout check of sensors

@dataclass
class MQTTState:
    mqtt_broker: str = "127.0.0.1"
    camera_topic: str = "camera/detections"
    pencil_topic: str = "pencil/reading"
    port: int = 1883
    client = None
    msg_count: int = 0

@dataclass
class CorrectionState:
    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0
    active_dz: bool = False
    est_z: float = 0.0

@dataclass
class RobotConfig:
    ip_address: str = '127.0.0.1'
    port: int = 5000
    socket = None
    timeout: float = 30.0 # TODO: adjust timeout as needed, maybe make it non-blocking with select instead
    connected: bool = False

class robotState:
    initial_pos : np.ndarray = None
    pos : np.ndarray = None
    orientation : np.ndarray = None

# --- GLOBALS ---
# MQTT_BROKER = "fe80::80ee:98fe:7fcb:95c3%16"
# MQTT_BROKER = "127.0.0.1"
MQTT_BROKER = "192.168.1.144"
WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480
img_center_x = WINDOW_WIDTH // 2
img_center_y = WINDOW_HEIGHT // 2

X_TARGET = 383.773
Y_TARGET = 4.28
Z_TARGET = -905.67
Z_THRESH = 4.0
MIN_PENCIL_Z = 0.25
PENCIL_Z_OFFSET = 40 # mm

canvas = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH, 3), dtype=np.uint8)
show = True

# --- LOGGERS ---
camera_logger = CSVLogger(name="camera", log_dir="test_logs")
pencil_logger = CSVLogger(name="pencil", log_dir="test_logs")
robot_logger = CSVLogger(name="robot", log_dir="test_logs")
diff_logger = CSVLogger(name="diff", log_dir="test_logs")

# --- CONFIGS ---
subscriber = MQTTState(mqtt_broker=MQTT_BROKER)
robot_config = RobotConfig()
