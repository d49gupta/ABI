from scripts.logger import CSVLogger
from enum import StrEnum
from dataclasses import dataclass
import numpy as np
from enum import Enum
from collections import deque
import threading
import time

PENCIL_TOPIC = "pencil/reading"
BINARY_PENCIL_TOPIC = "binary_pencil/reading"

# --- ENUMS ---
class CalibrationMode(Enum):
    FOUR_POINT = 0
    THREE_POINT = 1

class MotionState(Enum):
    IDLE = 0
    FIND_INIT_TAGS = 1
    FIND_TARGET = 2
    DESCEND = 3
    FIND_DEPTH = 4
    ASCEND = 5

class ThreePointState(StrEnum):
    FIND_CENTER = "camera/center_est"
    FIND_X = "camera/x_est"
    FIND_Y = "camera/y_est"
    IDLE = "camera"

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
    numb_tags: int = 0

@dataclass
class MQTTState:
    mqtt_broker: str = "127.0.0.1"
    camera_topic: str = "camera/center_est"
    pencil_topic: str = BINARY_PENCIL_TOPIC
    pi_topic: str = "pi/stop"
    pi_start_topic: str = "pi/start"
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
    port: int = 4000
    socket = None
    timeout: float = 20.0 # adjust timeout as needed, maybe make it non-blocking with select instead
    connected: bool = False
    msg_count: int = 0
    robot_file = None
    read_thread = None
    stop_trigger = None
    last_time = None
    last_dx: float = 0.0
    last_dy: float = 0.0
    last_dz: float = 0.0
    initial_pos : np.ndarray = None
    tcp_speed: float = 5.0
    init_est_z: float = 0.0

@dataclass
class conveyorState:
    running = False
    last_time = None

@dataclass
class robotState:
    pos : np.ndarray = None
    conveyor_axis: float = 0.0
    reported_speed: float = 0.0
    timestamp: int = 0

# --- CONFIG GLOBALS ---
MQTT_HOTSPOT_BROKER = "172.20.10.5"
MQTT_WIFI_BROKER = "192.168.0.54"
SIM_MQTT_BROKER = "127.0.0.1"
MQTT_BROKER = "10.89.1.194"
MQTT_ABI_BROKER = "10.89.1.159"
ROBOT_SIM_IP = "127.0.0.1"
ROBOT_REAL_IP = "10.60.70.51"
WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480
img_center_x = WINDOW_WIDTH // 2
img_center_y = WINDOW_HEIGHT // 2

# --- SIM GLOBALS --- 
X_TARGET = 545.692
Y_TARGET = 10.407
Z_TARGET = -905.672

# --- REAL GLOBALS --- 
Z_THRESH = 8.0
Z_TARGET_DEPTH = 4.0
Z_ACTIVE = 1.0
XY_TARGET_ACC = 1.0 # Minimum accuracy (mm) needed to enter descent state
Z_TARGET_ACC = 0.1
ASCENT_HEIGHT_DIFF = 5.0 # Diff between initial and ascent height
PENCIL_Z_OFFSET = 55.5 # mm between camera and pencil sensor on z axis
ROBOT_PUBLISH_RATE = 0
MOVE_DEADBAND_MM = 0.1 # suppress a MOVE_REL send if correction magnitude less than threshold
PENCIL_MOVE_RATE = 1.0 # slow movement of while in FIND_DEPTH state to not damange sensor
CONVEYOR_MOVE_TIME = 1.5

ASCENT_SPEED = 30.0
FIND_DEPTH_SPEED = 1.0
DESCENT_SPEED = 5.0
FIND_TARGET_SPEED = 15.0

# --- VISUALS --- 
canvas = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH, 3), dtype=np.uint8)
canvas_lock = threading.Lock()
show = True
show_camera_info = False

# --- BUFFERS ---
pencil_buffer = deque(maxlen=50)
correction_buffer = deque(maxlen=10)
robot_pose_buffer = deque(maxlen=25)

# --- SAMPLE STATES ---
correction = CorrectionState()
pencil_sample = pencilState()
camera_sample = cameraState()
robot_state = robotState()
conveyor_state = conveyorState()
state_last_time = time.perf_counter()

# --- LOGGERS ---
camera_logger = CSVLogger(name="camera", log_dir="current_logs")
pencil_logger = CSVLogger(name="pencil", log_dir="current_logs")
robot_logger = CSVLogger(name="robot", log_dir="current_logs")
correction_logger = CSVLogger(name="diff", log_dir="current_logs")
camera_perf_logger = CSVLogger(name="camera_perf", log_dir="current_logs")
controller_logger = CSVLogger(name="controller", log_dir="current_logs")
event_logger = CSVLogger(name="events", log_dir="current_logs")

# --- CONTROLLERS ---
alpha_camera = 0.5
smooth_dx = 0.0
smooth_dy = 0.0
Kp_target = 0.075
Kp_pencil = 0.1
Kp_ascent = 0.2
Kp_descent = 0.1

# --- STATES ---
class RobotState:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(RobotState, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, mode=None):
        if self._initialized: return
        
        self._initialized = True
        self.motion = MotionState.IDLE
        self.three_point = ThreePointState.IDLE
        self.calibration = mode
        self.recorded_points = []

        if self.calibration == CalibrationMode.FOUR_POINT:
            camera_topic = ThreePointState.FIND_CENTER.value
        else:
            camera_topic = self.three_point.value

        self.subscriber = MQTTState(mqtt_broker=MQTT_ABI_BROKER, camera_topic=camera_topic)
        self.robot_config = RobotConfig(ip_address=ROBOT_REAL_IP)

    def set_target(self, target):
        self.three_point = target
        self.subscriber.camera_topic = self.three_point.value

global_state = RobotState(CalibrationMode.FOUR_POINT)
# global_state.set_target(ThreePointState.FIND_CENTER)