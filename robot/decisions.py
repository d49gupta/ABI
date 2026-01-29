import paho.mqtt.client as mqtt
import json
import cv2
import numpy as np
from scripts.logger import CSVLogger
from dataclasses import dataclass

# --- DATACLASS ---
@dataclass
class PencilReading:
    raw: int
    millimeters: float
    flag: int

@dataclass # TODO: Add timestamp later
class CameraDetection:
    center_x: int
    center_y: int
    scale: float

# --- CONFIG ---
# MQTT_BROKER = "fe80::80ee:98fe:7fcb:95c3%16"
MQTT_BROKER = "127.0.0.1"
CAMERA_TOPIC = "camera/detections"
PENCIL_TOPIC = "pencil/reading"
WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480

# --- GLOBALS ---
canvas = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH, 3), dtype=np.uint8)
camera_logger = CSVLogger(name="camera", log_dir="../logs")
pencil_logger = CSVLogger(name="pencil", log_dir="../logs")
img_center_x = WINDOW_WIDTH // 2
img_center_y = WINDOW_HEIGHT // 2
pencil_offset_x = -50  # in mm
pencil_offset_y = 0    # in mm
dx = 0
dy = 0
dz = 0
pencil_sample = PencilReading(0, 0.0, 0)
camera_sample = CameraDetection(0, 0.0, 0)

def calculate_pencil_position():
    global dx, dy, dz
    if camera_sample.center_x == 0 and camera_sample.center_y == 0:
        return None

    scale = camera_sample.scale
    dx = (camera_sample.center_x - img_center_x) * scale + pencil_offset_x
    dy = (camera_sample.center_y - img_center_y) * scale + pencil_offset_y
    dz = pencil_sample.millimeters
    print(f"Pencil Position (mm): x={dx:.2f}, y={dy:.2f}, z={dz:.2f}")


def on_connect(client, userdata, flags, rc):
    print(f"Connected to Pi with result code {rc}")
    client.subscribe(CAMERA_TOPIC)
    client.subscribe(PENCIL_TOPIC)

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8")
        if msg.topic == PENCIL_TOPIC:
            receivePencil(payload)
        elif msg.topic == CAMERA_TOPIC:
            receiveCameraTemp(payload)

    except Exception as e:
            print(f"Error processing message on {msg.topic}: {e}")

def receivePencil(payload):
    data = json.loads(payload)
    raw = int(data["raw"])
    distance = float(data["millimeters"])
    flag = int(data["flag"])

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


def receiveCamera(payload):
    global canvas
    data = json.loads(payload)
    canvas.fill(0) 

    tags = data.get("tags", [])
    num_tags = len(tags)

    sum_cx = 0
    sum_cy = 0

    for tag in tags:
        x = int(tag["x"])
        y = int(tag["y"])
        tag_id = tag["id"]
        scale = float(tag["scale"])

        cv2.circle(canvas, (x, y), 8, (0, 255, 0), -1)
        cv2.putText(canvas, f"ID: {tag_id}", (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 255, 255), 1)
        
        sum_cx += tag["center_x"]
        sum_cy += tag["center_y"]

    if num_tags > 0:
        avg_cx = int(sum_cx / num_tags)
        avg_cy = int(sum_cy / num_tags)

        cv2.circle(canvas, (avg_cx, avg_cy), 12, (0, 0, 255), 2)
        cv2.circle(canvas, (avg_cx, avg_cy), 4, (0, 0, 255), -1)
        cv2.putText(canvas, "TARGET CENTER", (avg_cx + 15, avg_cy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        camera_logger.info("%.3f, %.3f, %.3f", avg_cx, avg_cy, scale)
        camera_sample.scale = scale
        camera_sample.center_x = avg_cx
        camera_sample.center_y = avg_cy

    print(f"Received {num_tags} tags. Center: ({avg_cx if num_tags > 0 else 0}, {avg_cy if num_tags > 0 else 0})")

if __name__ == "__main__":
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Connecting to {MQTT_BROKER}...")
    client.connect(MQTT_BROKER, 1883, 60)
    client.loop_start()

    while True:
        calculate_pencil_position()
        # cv2.imshow("AprilTag Real-Time Map", canvas)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    client.loop_stop()