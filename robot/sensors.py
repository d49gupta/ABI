from xmlrpc import client
import paho.mqtt.client as mqtt
import json
import cv2
import statistics
from robot.globals import *
from dataclasses import replace
import math

# Define offset in mm 
# Dont even need offset, just set target of pencil constant offset from center of camera target
# This way April Tags are always in view of camera no matter where pencil is
pencil_offset_x = 0
pencil_offset_y = 0

# Physical offsets from center of tag 4 to center of tags 5 and 6 (mm)
# Tag 5 is directly above tag 4 (-Y in image coords)
# Tag 6 is directly left of tag 4 (-X in image coords)
TAG5_OFFSET_X =   0.0
TAG5_OFFSET_Y = -78.5
TAG6_OFFSET_X = -78.5
TAG6_OFFSET_Y =   0.0

# 3-point calibration targets — updated by receive3ptCalibration()
# (pred_x, pred_y) in pixel space, averaged from tag 4 offset + tag own center
tag5_target = None
tag5_scale  = None
tag6_target = None
tag6_scale  = None

def on_connect(client, userdata, flags, rc):
    print(f"Connected to Pi with result code {rc}")
    client.subscribe(subscriber.camera_topic)
    client.subscribe(subscriber.pencil_topic)
    client.subscribe(subscriber.three_point_topic)

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8")
        subscriber.msg_count += 1
        if msg.topic == subscriber.pencil_topic:
            if subscriber.mqtt_broker == SIM_MQTT_BROKER:
                receivePencilSim(payload)
            else:
                receivePencil(payload)
        elif msg.topic == subscriber.camera_topic:
            if subscriber.mqtt_broker == SIM_MQTT_BROKER:
                receiveCameraSim(payload)
            else:
                receiveCamera(payload)
        elif msg.topic == subscriber.three_point_topic:
            receive3Point(payload)

    except Exception as e:
            print(f"Error processing message on {msg.topic}: {e}")

def connection_status():
    return subscriber.msg_count > 0

def receivePencil(payload):
    data = json.loads(payload)
    raw = int(data["raw"])
    distance = float(data["millimeters"])

    pencil_sample.raw = raw
    pencil_sample.distance = distance
    pencil_sample.active = distance >= Z_ACTIVE
    timestamp = time.perf_counter() - subscriber.start_time
    pencil_sample.timestamp = timestamp

    pencil_logger.info("%d, %.4f, %d", raw, distance, pencil_sample.active)
    curr_pencil_sample = replace(pencil_sample)
    pencil_buffer.append(curr_pencil_sample)

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

def receive3Point(payload):
    global canvas
    canvas.fill(0)

    data = json.loads(payload)
    x_tags = data.get("tags_x", [])
    y_tags = data.get("tags_y", [])

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
            camera_perf_logger.info("%d, %.4f, %.4f, %.4f, %.4f", num_tags, cv_cx, cv_cy, cv_scale, cv_z)
        
        timestamp = time.perf_counter() - subscriber.start_time
        camera_sample.scale = avg_scale # mm / px
        camera_sample.center_x = int(avg_cx)
        camera_sample.center_y = int(avg_cy)
        camera_sample.timestamp = timestamp
        camera_sample.num_tags = num_tags

        correction.dy  = (avg_cx - img_center_x) * camera_sample.scale - pencil_offset_x
        correction.dx = (avg_cy - img_center_y) * camera_sample.scale - pencil_offset_y
        correction.dz = avg_dz
        correction.timestamp = timestamp

        projected_x = int(int(WINDOW_WIDTH / 2) + pencil_offset_x / avg_scale)
        projected_y = int(int(WINDOW_HEIGHT / 2) + pencil_offset_y / avg_scale)
        camera_logger.info("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
                           avg_cx, avg_cy, avg_scale, correction.dx, correction.dy, correction.dz)
        
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
    else:
        camera_logger.warning("No detected tags")

def receive3ptCalibration(payload):
    """
    Processes 3-point calibration tag data (IDs 4, 5, 6) from the
    camera/3pt_calibration MQTT topic. Computes predicted pixel-space
    targets for tags 5 and 6 by averaging the prediction from tag 4's
    known physical offset with each tag's own homography center if visible.
    Updates module-level tag5_target, tag5_scale, tag6_target, tag6_scale.

    Args:
        payload: A JSON string containing a list of "tags" with id, center_x,
                 center_y, scale, and est_z fields.

    Returns:
        N/A

    Raises:
        JSONDecodeError: If the payload is not a valid JSON string.
        KeyError: If expected keys are missing from the payload.
    """
    global tag5_target, tag5_scale, tag6_target, tag6_scale

    data = json.loads(payload)
    tags = data.get("tags", [])

    ref  = next((t for t in tags if t["id"] == 4), None)
    tag5 = next((t for t in tags if t["id"] == 5), None)
    tag6 = next((t for t in tags if t["id"] == 6), None)

    if ref is None:
        # Cannot compute predictions without the reference tag
        return

    ref_cx = ref["center_x"]
    ref_cy = ref["center_y"]
    scale  = ref["scale"]  # mm per pixel

    # Predicted center of tag 5 from tag 4 + known physical offset
    pred5_from_tag4_x = ref_cx + TAG5_OFFSET_X / scale
    pred5_from_tag4_y = ref_cy + TAG5_OFFSET_Y / scale

    if tag5:
        # Average tag 4 prediction with tag 5's own homography center
        pred5_x = (pred5_from_tag4_x + tag5["center_x"]) / 2
        pred5_y = (pred5_from_tag4_y + tag5["center_y"]) / 2
        tag5_scale = (scale + tag5["scale"]) / 2
    else:
        # Fall back to tag 4 prediction only
        pred5_x = pred5_from_tag4_x
        pred5_y = pred5_from_tag4_y
        tag5_scale = scale

    tag5_target = (pred5_x, pred5_y)

    # Predicted center of tag 6 from tag 4 + known physical offset
    pred6_from_tag4_x = ref_cx + TAG6_OFFSET_X / scale
    pred6_from_tag4_y = ref_cy + TAG6_OFFSET_Y / scale

    if tag6:
        # Average tag 4 prediction with tag 6's own homography center
        pred6_x = (pred6_from_tag4_x + tag6["center_x"]) / 2
        pred6_y = (pred6_from_tag4_y + tag6["center_y"]) / 2
        tag6_scale = (scale + tag6["scale"]) / 2
    else:
        # Fall back to tag 4 prediction only
        pred6_x = pred6_from_tag4_x
        pred6_y = pred6_from_tag4_y
        tag6_scale = scale

    tag6_target = (pred6_x, pred6_y)

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
    subscriber.start_time = time.perf_counter()

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