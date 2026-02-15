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
    est_z: float = 0.0

# --- GLOBALS ---
# MQTT_BROKER = "fe80::80ee:98fe:7fcb:95c3%16"
# MQTT_BROKER = "127.0.0.1"
MQTT_BROKER = "192.168.1.144"

WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480
camera_logger = CSVLogger(name="camera", log_dir="test_logs")
pencil_logger = CSVLogger(name="pencil", log_dir="test_logs")
img_center_x = WINDOW_WIDTH // 2
img_center_y = WINDOW_HEIGHT // 2

# Define offset in mm 
# Dont even need offset, just set target of pencil constant offset from center of camera target
# This way April Tags are always in view of camera no matter where pencil is
pencil_offset_x = 0
pencil_offset_y = 0
canvas = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH, 3), dtype=np.uint8)
show = True

# --- STATE ---
correction = CorrectionState()
subscriber = MQTTState(mqtt_broker=MQTT_BROKER)
pencil_sample = PencilReading()
camera_sample = CameraDetection()

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
            receiveCamera(payload)

    except Exception as e:
            print(f"Error processing message on {msg.topic}: {e}")

def receivePencil(payload):
    data = json.loads(payload)
    raw = int(data["raw"])
    distance = float(data["millimeters"])
    flag = int(data["flag"])

    if abs(distance) < 0.25:
        pencil_sample.active = False
    else:        
        pencil_sample.active = True

    pencil_logger.info("%d, %.4f, %d", raw, distance, flag)
    pencil_sample.raw = raw
    pencil_sample.millimeters = distance
    pencil_sample.flag = flag

    correction.dz = pencil_sample.millimeters
    correction.active_dz = pencil_sample.active
    print(f"Pencil Distance (mm): {correction.dz:.2f}, Active: {correction.active_dz}")

def receiveCameraTemp(payload):
    data = json.loads(payload)
    center_x = int(data["center_x"])
    center_y = int(data["center_y"])
    camera_sample.center_x = center_x
    camera_sample.center_y = center_y
    camera_sample.scale = 0.05  # Temporary fixed scale
    correction.dx = camera_sample.center_x * camera_sample.scale
    correction.dy = 0
    print(f"Received Camera center: ({center_x}, {center_y})")

def receiveCamera(payload):
    global canvas
    canvas.fill(0)

    data = json.loads(payload)
    tags = data.get("tags", [])
    num_tags = len(tags)

    sum_cx = 0
    sum_cy = 0
    sum_scale = 0
    sum_est_z = 0

    for tag in tags:
        x = int(tag["x"])
        y = int(tag["y"])
        tag_id = tag["id"]
        scale = float(tag["scale"])
        sum_cx += tag["center_x"]
        sum_cy += tag["center_y"]
        sum_scale += scale
        sum_est_z += tag["est_z"]

        if show:
            cv2.circle(canvas, (x, y), 8, (0, 255, 0), -1)
            cv2.putText(canvas, f"ID: {tag_id}", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 255, 255), 1)

    if num_tags > 0:
        avg_cx = int(sum_cx / num_tags)
        avg_cy = int(sum_cy / num_tags)
        avg_scale = sum_scale / num_tags
        avg_est_z = sum_est_z / num_tags
        
        camera_sample.scale = avg_scale
        camera_sample.center_x = avg_cx
        camera_sample.center_y = avg_cy

        if camera_sample.center_x == 0 and camera_sample.center_y == 0:
            return None

        scale = camera_sample.scale # mm / px
        correction.est_z = avg_est_z
        correction.dx  = (camera_sample.center_x - img_center_x) * scale - pencil_offset_x
        correction.dy = (camera_sample.center_y - img_center_y) * scale - pencil_offset_y
        projected_x = int(int(WINDOW_WIDTH / 2) + pencil_offset_x / avg_scale)
        projected_y = int(int(WINDOW_HEIGHT / 2) + pencil_offset_y / avg_scale)
        camera_logger.info("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
                           avg_cx, avg_cy, avg_scale, correction.dx, correction.dy, correction.est_z, projected_x, projected_y)

        if show:
            cv2.circle(canvas, (projected_x, projected_y), 5, (255, 0, 255), -1)
            cv2.putText(canvas, "TIP", (projected_x + 10, projected_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 0, 255), 1)
            
            # Putting the target circle on the canvas and logging the predicted point on the image in px
            cv2.circle(canvas, (avg_cx, avg_cy), 12, (0, 0, 255), 2)
            cv2.circle(canvas, (avg_cx, avg_cy), 4, (0, 0, 255), -1)
            inv_scale = 1 / avg_scale
            cv2.putText(canvas, f"TARGET CENTER: {inv_scale:.2f} pixel/mm", (avg_cx + 15, avg_cy + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(canvas, f"ESTIMATED DEPTH: {avg_est_z:.2f} mm", (75, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

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

if __name__ == "__main__":
    connect_sensors()
    start_sensors()

    while True:
        cv2.imshow("AprilTag Real-Time Map", canvas)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    stop_sensors()