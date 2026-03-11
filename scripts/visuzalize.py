import paho.mqtt.client as mqtt
import json
import cv2
import numpy as np
from logger import CSVLogger

# MQTT_BROKER = "192.168.0.43"
# MQTT_BROKER = "172.20.10.5" # Hotspot
MQTT_BROKER = "10.0.0.175" # Evan Home Wifi
# MQTT_BROKER = "127.0.0.1"
CAMERA_TOPIC = "camera/detections"
PENCIL_TOPIC = "pencil/reading"
THREE_PT_TOPIC = "camera/3pt_calibration"
WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480

canvas = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH, 3), dtype=np.uint8)
camera_logger = CSVLogger(name="camera", log_dir="../test_logs")
pencil_logger = CSVLogger(name="pencil", log_dir="../test_logs")

# Mechanically fixed offsets from camera to pencil (cm)
endpoint_offset_x = 0
endpoint_offset_y = 0

# Physical offsets from center of tag 4 to center of tags 5 and 6 (mm)
# Tag 5 is directly above tag 4 (-Y in image coords)
# Tag 6 is directly left of tag 4 (-X in image coords)
TAG5_OFFSET_X =   0.0
TAG5_OFFSET_Y = -78.5
TAG6_OFFSET_X = 78.5
TAG6_OFFSET_Y =   0.0

# Cached latest data from each MQTT topic
latest_camera_data = []
latest_3pt_data = []

def on_connect(client, userdata, flags, rc):
    """
    When connected to the board setup subscribers to the camera module,
    the digital pressure sensor, and the 3-point calibration topic.

    Args:
        client: The client instance for this runtime
        userdata:
        flags:
        rc: The result code response of the connection

    Returns:
        N/A

    Raises:
        N/A
    """
    print(f"Connected to Pi with result code {rc}")
    client.subscribe(CAMERA_TOPIC)
    client.subscribe(PENCIL_TOPIC)
    client.subscribe(THREE_PT_TOPIC)

def on_message(client, userdata, msg):
    """
    Callback triggered when a message is received from the MQTT broker.
    Routes the payload to specific processing functions based on the topic.

    Args:
        client: The client instance for this callback.
        userdata: The private user data as set in Client() or user_data_set().
        msg: An instance of MQTTMessage. This is a class with members topic,
             payload, qos, retain.

    Returns:
        N/A

    Raises:
        Exception: Logged to console if message decoding or routing fails.
    """
    try:
        payload = msg.payload.decode("utf-8")
        if msg.topic == PENCIL_TOPIC:
            receivePencil(payload)
        elif msg.topic == CAMERA_TOPIC:
            receiveCamera(payload)
        elif msg.topic == THREE_PT_TOPIC:
            receive3ptCalibration(payload)

    except Exception as e:
        print(f"Error processing message on {msg.topic}: {e}")

def receivePencil(payload):
    """
    Parses digital pressure sensor data, logs the raw and physical values to
    a CSV file, and prints status to the console.

    Args:
        payload: A JSON string containing "raw", "millimeters", and "flag".

    Returns:
        N/A

    Raises:
        JSONDecodeError: If the payload is not a valid JSON string.
        KeyError: If expected keys are missing from the payload.
    """
    data = json.loads(payload)
    raw = int(data["raw"])
    distance = float(data["millimeters"])
    flag = int(data["flag"])
    pencil_logger.info("%d, %.4f, %d", raw, distance, flag)
    print(f"Received Pencil reading: {raw} bits")

def receiveCamera(payload):
    """
    Caches the latest 4-point AprilTag detection data (IDs 0-4) received
    from the camera/detections MQTT topic. Drawing is handled by drawCanvas()
    in the main loop to avoid canvas conflicts with receive3ptCalibration().

    Args:
        payload: A JSON string containing a list of "tags", each with x, y,
                 id, center_x, center_y, scale, and est_z.

    Returns:
        N/A

    Raises:
        JSONDecodeError: If the payload is not a valid JSON string.
    """
    global latest_camera_data
    data = json.loads(payload)
    latest_camera_data = data.get("tags", [])
    print(f"Received {len(latest_camera_data)} 4pt tags")

def receive3ptCalibration(payload):
    """
    Caches the latest 3-point calibration tag data (IDs 4, 5, 6) received
    from the camera/3pt_calibration MQTT topic. Drawing is handled by
    drawCanvas() in the main loop to avoid canvas conflicts with
    receiveCamera().

    Args:
        payload: A JSON string containing a list of "tags" with id, center_x,
                 center_y, scale, and est_z fields.

    Returns:
        N/A

    Raises:
        JSONDecodeError: If the payload is not a valid JSON string.
    """
    global latest_3pt_data
    data = json.loads(payload)
    latest_3pt_data = data.get("tags", [])
    print(f"Received {len(latest_3pt_data)} 3pt tags")

def drawCanvas():
    """
    Clears the canvas and redraws all tag data from both MQTT topics each
    frame. 4-point tags (IDs 0-4) are drawn in green with their averaged
    center estimate in red. 3-point tags (IDs 5, 6) are drawn in orange
    showing where the camera sees them, and red circle targets show the
    predicted centers computed from tag 4's position and known physical
    offsets.

    Returns:
        N/A
    """
    global canvas
    canvas.fill(0)

    # 4-point calibration tags
    sum_cx = 0
    sum_cy = 0
    sum_scale = 0
    avg_cx = 0
    avg_cy = 0
    num_tags = len(latest_camera_data)

    for tag in latest_camera_data:
        x, y = int(tag["x"]), int(tag["y"])
        tag_id = tag["id"]

        # Raw tag centroid
        cv2.circle(canvas, (x, y), 8, (0, 255, 0), -1)
        cv2.putText(canvas, f"ID: {tag_id}", (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 255, 255), 1)

        # Projected center point
        cv2.circle(canvas, (int(tag["center_x"]), int(tag["center_y"])), 4, (0, 255, 0), -1)
        cv2.putText(canvas, f"ID: {tag_id}", (int(tag["center_x"]) + 10, int(tag["center_y"]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 0, 255), 1)

        sum_cx += tag["center_x"]
        sum_cy += tag["center_y"]
        sum_scale += tag["scale"]

    if num_tags > 0:
        avg_cx = int(sum_cx / num_tags)
        avg_cy = int(sum_cy / num_tags)
        avg_scale = sum_scale / num_tags

        projected_x = int(WINDOW_WIDTH / 2 + endpoint_offset_x / avg_scale)
        projected_y = int(WINDOW_HEIGHT / 2 + endpoint_offset_y / avg_scale)

        # Pencil tip position
        cv2.circle(canvas, (projected_x, projected_y), 5, (255, 0, 255), -1)
        cv2.putText(canvas, "TIP", (projected_x + 10, projected_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 0, 255), 1)

        # 4-point target center
        cv2.circle(canvas, (avg_cx, avg_cy), 12, (0, 0, 255), 2)
        cv2.circle(canvas, (avg_cx, avg_cy), 4,  (0, 0, 255), -1)
        inv_scale = 1 / avg_scale
        cv2.putText(canvas, f"TARGET CENTER: {inv_scale:.2f} pixel/cm", (avg_cx + 15, avg_cy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        camera_logger.info("%.3f, %.3f", avg_cx, avg_cy)

    # 3-point calibration tags
    ref  = next((t for t in latest_3pt_data if t["id"] == 4), None)
    tag5 = next((t for t in latest_3pt_data if t["id"] == 5), None)
    tag6 = next((t for t in latest_3pt_data if t["id"] == 6), None)

    # Draw tag 4 reference center in yellow
    if ref:
        ref_cx = ref["center_x"]
        ref_cy = ref["center_y"]
        scale  = ref["scale"]  # mm per pixel

        cv2.circle(canvas, (int(ref_cx), int(ref_cy)), 6, (0, 255, 255), -1)
        cv2.putText(canvas, "REF ID:4", (int(ref_cx) + 10, int(ref_cy) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)

       # Predicted target for tag 5
        pred5_from_tag4_x = ref_cx + TAG5_OFFSET_X / scale
        pred5_from_tag4_y = ref_cy + TAG5_OFFSET_Y / scale

        if tag5:
            # Average tag 4 prediction with tag 5's own homography center
            pred5_x = int((pred5_from_tag4_x + tag5["center_x"]) / 2)
            pred5_y = int((pred5_from_tag4_y + tag5["center_y"]) / 2)
        else:
            # Fall back to tag 4 prediction only
            pred5_x = int(pred5_from_tag4_x)
            pred5_y = int(pred5_from_tag4_y)

        cv2.circle(canvas, (pred5_x, pred5_y), 12, (0, 0, 255), 2)
        cv2.circle(canvas, (pred5_x, pred5_y), 4,  (0, 0, 255), -1)
        cv2.putText(canvas, "PRED ID:5", (pred5_x + 15, pred5_y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        # Predicted target for tag 6
        pred6_from_tag4_x = ref_cx + TAG6_OFFSET_X / scale
        pred6_from_tag4_y = ref_cy + TAG6_OFFSET_Y / scale

        if tag6:
            # Average tag 4 prediction with tag 6's own homography center
            pred6_x = int((pred6_from_tag4_x + tag6["center_x"]) / 2)
            pred6_y = int((pred6_from_tag4_y + tag6["center_y"]) / 2)
        else:
            # Fall back to tag 4 prediction only
            pred6_x = int(pred6_from_tag4_x)
            pred6_y = int(pred6_from_tag4_y)

        cv2.circle(canvas, (pred6_x, pred6_y), 12, (0, 0, 255), 2)
        cv2.circle(canvas, (pred6_x, pred6_y), 4,  (0, 0, 255), -1)
        cv2.putText(canvas, "PRED ID:6", (pred6_x + 15, pred6_y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    # Draw where the camera directly sees tags 5 and 6
    for tag in [tag5, tag6]:
        if tag:
            x,  y  = int(tag["x"]),        int(tag["y"])
            cx, cy = int(tag["center_x"]), int(tag["center_y"])

            # Raw seen position
            cv2.circle(canvas, (x, y), 8, (0, 165, 255), -1)
            cv2.putText(canvas, f"3PT ID:{tag['id']}", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

            # Homography center
            cv2.circle(canvas, (cx, cy), 4, (0, 165, 255), -1)

            print(f"SEEN (ID {tag['id']}): center=({tag['center_x']:.1f}, {tag['center_y']:.1f})  scale={tag['scale']:.4f}")

    print(f"4pt tags: {num_tags}  Center: ({avg_cx}, {avg_cy})")

if __name__ == "__main__":
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Connecting to {MQTT_BROKER}...")
    client.connect(MQTT_BROKER, 1883, 60)
    client.loop_start()

    while True:
        drawCanvas()
        cv2.imshow("AprilTag Real-Time Map", canvas)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    client.loop_stop()