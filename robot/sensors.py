from xmlrpc import client
import paho.mqtt.client as mqtt
import json
import cv2
import statistics
from robot.globals import *

# Define offset in mm 
# Dont even need offset, just set target of pencil constant offset from center of camera target
# This way April Tags are always in view of camera no matter where pencil is
pencil_offset_x = 0
pencil_offset_y = 0

# --- STATES ---
correction = CorrectionState()
pencil_sample = PencilReading()
camera_sample = CameraDetection()

def on_connect(client, userdata, flags, rc):
    print(f"Connected to Pi with result code {rc}")
    client.subscribe(subscriber.camera_topic)
    client.subscribe(subscriber.pencil_topic)

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8")
        subscriber.msg_count += 1
        if msg.topic == subscriber.pencil_topic:
            receivePencil(payload)
        elif msg.topic == subscriber.camera_topic:
            receiveCamera(payload)

    except Exception as e:
            print(f"Error processing message on {msg.topic}: {e}")

def connection_status():
    return subscriber.msg_count > 0

def receivePencil(payload):
    data = json.loads(payload)
    raw = int(data["raw"])
    distance = float(data["millimeters"])

    pencil_logger.info("%d, %.4f", raw, distance)
    pencil_sample.raw = raw
    pencil_sample.millimeters = distance
    correction.dz = pencil_sample.millimeters

    if abs(distance) < MIN_PENCIL_Z:
        pencil_sample.active = False
        correction.active_dz = pencil_sample.active
    else:        
        pencil_sample.active = True
        correction.active_dz = pencil_sample.active
        print("PENCIL SENSOR ACTIVE")

    # print(f"Pencil Distance (mm): {correction.dz:.2f}, Active: {correction.active_dz}")

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

    cx_list = []
    cy_list = []
    scale_list = []
    z_list = []

    for tag in tags:
        x = int(tag["x"])
        y = int(tag["y"])
        tag_id = tag["id"]

        scale_list.append(float(tag["scale"]))
        cx_list.append(tag["center_x"])
        cy_list.append(tag["center_y"])
        z_list.append(tag["est_z"])

        if show:
            cv2.circle(canvas, (x, y), 8, (0, 255, 0), -1)
            cv2.putText(canvas, f"ID: {tag_id}", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 255, 255), 1)

    if num_tags > 0:
        avg_cx = statistics.mean(cx_list)
        avg_cy = statistics.mean(cy_list)
        avg_scale = statistics.mean(scale_list)
        avg_est_z = statistics.mean(z_list)

        if num_tags > 1 and show_camera_info:
            cv_cx = (statistics.stdev(cx_list) / avg_cx) if avg_cx != 0 else 0
            cv_cy = (statistics.stdev(cy_list) / avg_cy) if avg_cy != 0 else 0
            cv_scale = (statistics.stdev(scale_list) / avg_scale) if avg_scale != 0 else 0
            cv_z = (statistics.stdev(z_list) / avg_est_z) if avg_est_z != 0 else 0
            camera_perf.info("%d, %.4f, %.4f, %.4f, %.4f", num_tags, cv_cx, cv_cy, cv_scale, cv_z)
        
        camera_sample.scale = avg_scale # mm / px
        camera_sample.center_x = int(avg_cx)
        camera_sample.center_y = int(avg_cy)
        correction.est_z = avg_est_z

        correction.dx  = (avg_cx - img_center_x) * camera_sample.scale - pencil_offset_x
        correction.dy = (avg_cy - img_center_y) * camera_sample.scale - pencil_offset_y
        projected_x = int(int(WINDOW_WIDTH / 2) + pencil_offset_x / avg_scale)
        projected_y = int(int(WINDOW_HEIGHT / 2) + pencil_offset_y / avg_scale)
        camera_logger.info("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
                           avg_cx, avg_cy, avg_scale, correction.dx, correction.dy, correction.est_z)

        if show:
            cv2.circle(canvas, (projected_x, projected_y), 5, (255, 0, 255), -1)
            cv2.putText(canvas, "TIP", (projected_x + 10, projected_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 0, 255), 1)
            
            # Putting the target circle on the canvas and logging the predicted point on the image in px
            cv2.circle(canvas, (camera_sample.center_x, camera_sample.center_y), 12, (0, 0, 255), 2)
            cv2.circle(canvas, (camera_sample.center_x, camera_sample.center_y), 4, (0, 0, 255), -1)
            inv_scale = 1 / avg_scale
            cv2.putText(canvas, f"TARGET CENTER: {inv_scale:.2f} pixel/mm", (camera_sample.center_x + 15, camera_sample.center_y + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(canvas, f"ESTIMATED DEPTH: {avg_est_z:.2f} mm", (75, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # print(f"Received {num_tags} tags. Center: ({avg_cx if num_tags > 0 else 0}, {avg_cy if num_tags > 0 else 0})")

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