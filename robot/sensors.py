from xmlrpc import client
import paho.mqtt.client as mqtt
import json
import cv2
import statistics
from robot.globals import *
from dataclasses import replace
import math
from robot.test import global_state as state

# Define offset in mm 
# Dont even need offset, just set target of pencil constant offset from center of camera target
# This way April Tags are always in view of camera no matter where pencil is
pencil_offset_x = 0
pencil_offset_y = 0

def on_connect(client, userdata, flags, rc):
    print(f"Connected to Pi with result code {rc}")
    client.subscribe(state.subscriber.camera_topic)
    client.subscribe(state.subscriber.pencil_topic)

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8")
        state.subscriber.msg_count += 1
        print(state.calibration, state.subscriber.camera_topic, msg.topic)
        
        if msg.topic == state.subscriber.pencil_topic:
            if state.subscriber.mqtt_broker == SIM_MQTT_BROKER:
                receivePencilSim(payload)
            else:
                receivePencil(payload)
        elif (state.calibration == CalibrationMode.FOUR_POINT and msg.topic == ThreePointState.FIND_CENTER.value) or \
            (state.calibration == CalibrationMode.THREE_POINT and msg.topic == state.subscriber.camera_topic): 
            print("DSAJDHASDHAS")
            if state.subscriber.mqtt_broker == SIM_MQTT_BROKER:
                receiveCameraSim(payload)
            else:
                receiveCamera(payload)
    except Exception as e:
            print(f"Error processing message on {msg.topic}: {e}")

def connection_status():
    return state.subscriber.msg_count > 0

def receivePencil(payload):
    data = json.loads(payload)
    raw = int(data["raw"])
    distance = float(data["millimeters"])

    pencil_sample.raw = raw
    pencil_sample.distance = distance
    pencil_sample.active = distance >= Z_ACTIVE
    timestamp = time.perf_counter() - state.subscriber.start_time
    pencil_sample.timestamp = timestamp

    pencil_logger.info("%d, %d, %.4f, %d", state.motion.value, raw, distance, pencil_sample.active)
    curr_pencil_sample = replace(pencil_sample)
    pencil_buffer.append(curr_pencil_sample)

    # print(f"Pencil Distance (mm): {correction.dz:.2f}")

def receivePencilSim(payload):
    data = json.loads(payload)
    pencil_sample.timestamp = int(data["ms"])
    pencil_sample.distance = float(data["dist"])
    curr_pencil_sample = replace(pencil_sample)
    pencil_buffer.append(curr_pencil_sample)

def receiveCameraSim(payload):
    data = json.loads(payload)
    correction.timestamp = int(data["ms"])
    correction.dx = int(data["correction_x"])
    correction.dy = int(data["correction_y"])
    curr_correction_sample = replace(correction)
    correction_buffer.append(curr_correction_sample)

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
            with canvas_lock:
                cv2.circle(canvas, (x, y), 8, (0, 255, 0), -1)
                cv2.putText(canvas, f"ID: {tag_id}", (x + 10, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 255, 255), 1)

    if num_tags > 0:
        avg_cx = statistics.mean(cx_list)
        avg_cy = statistics.mean(cy_list)
        avg_scale = statistics.mean(scale_list)
        avg_dz = statistics.mean(z_list)

        if num_tags > 1 and show_camera_info:
            cv_cx = (statistics.stdev(cx_list) / avg_cx) if avg_cx != 0 else 0
            cv_cy = (statistics.stdev(cy_list) / avg_cy) if avg_cy != 0 else 0
            cv_scale = (statistics.stdev(scale_list) / avg_scale) if avg_scale != 0 else 0
            cv_z = (statistics.stdev(z_list) / avg_dz) if avg_dz != 0 else 0
            camera_perf_logger.info("%d, %d, %.4f, %.4f, %.4f, %.4f", state.motion.value, num_tags, cv_cx, cv_cy, cv_scale, cv_z)
        
        timestamp = time.perf_counter() - state.subscriber.start_time
        camera_sample.scale = avg_scale # mm / px
        camera_sample.center_x = int(avg_cx)
        camera_sample.center_y = int(avg_cy)
        camera_sample.timestamp = timestamp
        camera_sample.num_tags = num_tags

        correction.dy  = (avg_cx - img_center_x) * camera_sample.scale - pencil_offset_x
        correction.dx = (avg_cy - img_center_y) * camera_sample.scale - pencil_offset_y
        # correction.dz = avg_dz
        correction.dz = math.sqrt(abs(avg_dz ** 2 - correction.dy ** 2 - correction.dx ** 2))
        correction.timestamp = timestamp

        projected_x = int(int(WINDOW_WIDTH / 2) + pencil_offset_x / avg_scale)
        projected_y = int(int(WINDOW_HEIGHT / 2) + pencil_offset_y / avg_scale)
        camera_logger.info("%d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
                           state.motion.value, avg_cx, avg_cy, avg_scale, correction.dx, correction.dy, correction.dz)
        
        curr_camera_sample = replace(camera_sample)
        curr_correction = replace(correction)
        camera_buffer.append(curr_camera_sample)
        correction_buffer.append(curr_correction)

        if show:
            with canvas_lock:
                cv2.circle(canvas, (projected_x, projected_y), 5, (255, 0, 255), -1)
                cv2.putText(canvas, "TIP", (projected_x + 10, projected_y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 0, 255), 1)
                
                cv2.circle(canvas, (curr_camera_sample.center_x, curr_camera_sample.center_y), 12, (0, 0, 255), 2)
                cv2.circle(canvas, (curr_camera_sample.center_x, curr_camera_sample.center_y), 4, (0, 0, 255), -1)
                inv_scale = 1 / avg_scale
                cv2.putText(canvas, f"TARGET CENTER: {inv_scale:.2f} pixel/mm", (curr_camera_sample.center_x + 15, curr_camera_sample.center_y + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.putText(canvas, f"ESTIMATED DEPTH: {avg_dz:.2f} mm", (75, 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # print(f"Received {num_tags} tags. Center: ({avg_cx if num_tags > 0 else 0}, {avg_cy if num_tags > 0 else 0})")
    else:
        camera_logger.warning("No detected tags")

def connect_sensors():
    state.subscriber.client = mqtt.Client()
    state.subscriber.client.on_connect = on_connect
    state.subscriber.client.on_message = on_message

    print(f"Connecting to {state.subscriber.mqtt_broker}...")
    state.subscriber.client.connect(state.subscriber.mqtt_broker, state.subscriber.port, 60)

def start_sensors():
    state.subscriber.client.loop_start()

def stop_sensors():
    # state.subscriber.client.publish(state.subscriber.pi_topic, "STOP")
    state.subscriber.client.loop_stop()
    state.subscriber.start_time = time.perf_counter()

if __name__ == "__main__":
    connect_sensors()
    start_sensors()

    while True:
        if show:
            with canvas_lock:
                cv2.imshow("AprilTag Real-Time Map", canvas)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    stop_sensors()