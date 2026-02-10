from xmlrpc import client
import paho.mqtt.client as mqtt
import json
import cv2
import numpy as np
from scripts.logger import CSVLogger
from dataclasses import dataclass

# --- DATACLASS ---
@dataclass
class PencilReading:
    raw: int = 0
    millimeters: float = 0.0
    flag: int = 0
    active: bool = False

@dataclass # TODO: Add timestamp later
class CameraDetection:
    center_x: int = 0
    center_y: int = 0
    scale: float = 0.0

@dataclass
class MQTTState:
    mqtt_broker: str = "127.0.0.1"
    camera_topic: str = "camera/detections"
    pencil_topic: str = "pencil/reading"
    port: int = 1883
    client = None

@dataclass
class CorrectionState:
    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0
    active_dz: bool = False

# --- GLOBALS ---
# MQTT_BROKER = "fe80::80ee:98fe:7fcb:95c3%16"
MQTT_BROKER = "127.0.0.1"
WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480
camera_logger = CSVLogger(name="camera", log_dir="test_logs")
pencil_logger = CSVLogger(name="pencil", log_dir="test_logs")
img_center_x = WINDOW_WIDTH // 2
img_center_y = WINDOW_HEIGHT // 2
pencil_offset_x = -50  # in mm
pencil_offset_y = 0    # in mm

# --- STATE ---
correction = CorrectionState
subscriber = MQTTState(mqtt_broker=MQTT_BROKER)
pencil_sample = PencilReading()
camera_sample = CameraDetection()

def calculate_xy_target():
    if camera_sample.center_x == 0 and camera_sample.center_y == 0:
        return None

    scale = camera_sample.scale
    correction.dx  = (camera_sample.center_x - img_center_x) * scale + pencil_offset_x
    correction.dy = (camera_sample.center_y - img_center_y) * scale + pencil_offset_y
    # print(f"Pencil Position (mm): x={correction.dx:.2f}, y={correction.dy:.2f}")

def calculate_z_target():
    correction.dz = pencil_sample.millimeters
    correction.active_dz = pencil_sample.active
    print(f"Pencil Distance (mm): {correction.dz:.2f}, Active: {correction.active_dz}")

def on_connect(client, userdata, flags, rc):
    print(f"Connected to Pi with result code {rc}")
    client.subscribe(subscriber.camera_topic)
    client.subscribe(subscriber.pencil_topic)

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8")
        if msg.topic == subscriber.pencil_topic:
            receivePencil(payload)
        elif msg.topic == subscriber.camera_topic:
            receiveCameraTemp(payload)

    except Exception as e:
            print(f"Error processing message on {msg.topic}: {e}")

def receivePencil(payload):
    data = json.loads(payload)
    raw = int(data["raw"])
    distance = float(data["millimeters"])
    flag = int(data["flag"])

    if abs(distance) < 0.05:
        pencil_sample.active = False
    else:        
        pencil_sample.active = True

    pencil_logger.info("%d, %.4f, %d", raw, distance, flag)
    pencil_sample.raw = raw
    pencil_sample.millimeters = distance
    pencil_sample.flag = flag
    print(f"Received Pencil reading: {raw} bits")

def receiveCameraTemp(payload):
    data = json.loads(payload)
    center_x = int(data["center_x"])
    center_y = int(data["center_y"])
    camera_sample.center_x = center_x
    camera_sample.center_y = center_y
    camera_sample.scale = 0.01  # Temporary fixed scale
    print(f"Received Camera center: ({center_x}, {center_y})")
    calculate_xy_target()

def receiveCamera(payload):
    data = json.loads(payload)
    tags = data.get("tags", [])
    num_tags = len(tags)

    sum_cx = 0
    sum_cy = 0

    for tag in tags:
        x = int(tag["x"])
        y = int(tag["y"])
        tag_id = tag["id"]
        scale = float(tag["scale"])
        sum_cx += tag["center_x"]
        sum_cy += tag["center_y"]

    if num_tags > 0:
        avg_cx = int(sum_cx / num_tags)
        avg_cy = int(sum_cy / num_tags)
        
        camera_logger.info("%.3f, %.3f, %.3f", avg_cx, avg_cy, scale)
        camera_sample.scale = scale
        camera_sample.center_x = avg_cx
        camera_sample.center_y = avg_cy

    print(f"Received {num_tags} tags. Center: ({avg_cx if num_tags > 0 else 0}, {avg_cy if num_tags > 0 else 0})")

def connect_sensors():
    subscriber.client = mqtt.Client()
    subscriber.client.on_connect = on_connect
    subscriber.client.on_message = on_message

    print(f"Connecting to {subscriber.mqtt_broker}...")
    subscriber.client.connect(subscriber.mqtt_broker, subscriber.port, 60)

def start_sensors():
    subscriber.client.loop_start()

def stop_sensors():
    subscriber.client.loop_stop()