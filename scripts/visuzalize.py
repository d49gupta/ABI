import paho.mqtt.client as mqtt
import json
import cv2
import numpy as np
from logger import CSVLogger

MQTT_BROKER = "192.168.1.144"
# MQTT_BROKER = "fe80::80ee:98fe:7fcb:95c3%16"
# MQTT_BROKER = "127.0.0.1"
CAMERA_TOPIC = "camera/detections"
PENCIL_TOPIC = "pencil/reading"
WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480

canvas = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH, 3), dtype=np.uint8)
camera_logger = CSVLogger(name="camera", log_dir="../test_logs")
pencil_logger = CSVLogger(name="pencil", log_dir="../test_logs")

# Mechanically fixed offsets from camera to pencil (cm)
endpoint_offset_x = -14
endpoint_offset_y = 0

def on_connect(client, userdata, flags, rc):
    """
    When connected to the board setup subscribers to the camera module and 
    the digital pressure sensor.

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

# TODO: Overlay this with a live compressed video feed
def receiveCamera(payload):
    """
    Processes AprilTag detection data to update the global canvas. Draws 
    individual tag positions and calculates/visualizes the average center point.

    Args:
        payload: A JSON string containing a list of "tags", each with x, y, 
                 id, center_x, and center_y.

    Returns:
        N/A
    
    Raises:
        JSONDecodeError: If the payload is not a valid JSON string.
        ZeroDivisionError: Handled internally if no tags are detected.
    """
    global canvas
    data = json.loads(payload)
    canvas.fill(0) 

    tags = data.get("tags", [])
    num_tags = len(tags)

    sum_cx = 0
    sum_cy = 0
    sum_scale = 0

    for tag in tags:
        x = int(tag["x"])
        y = int(tag["y"])
        tag_id = tag["id"]

        cv2.circle(canvas, (x, y), 8, (0, 255, 0), -1)
        cv2.putText(canvas, f"ID: {tag_id}", (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 255, 255), 1)
        
        sum_cx += tag["center_x"]
        sum_cy += tag["center_y"]
        sum_scale += tag["scale"]

    if num_tags > 0:
        avg_cx = int(sum_cx / num_tags)
        avg_cy = int(sum_cy / num_tags)
        avg_scale = sum_scale / num_tags

        projected_x = int(int(WINDOW_WIDTH / 2) + endpoint_offset_x / avg_scale)
        projected_y = int(int(WINDOW_HEIGHT / 2) + endpoint_offset_y / avg_scale) # make sure camera and body directions are consistent

        # Plotting the relation between the camera and the predicted center 
        cv2.circle(canvas, (projected_x, projected_y), 5, (255, 0, 255), -1)
        cv2.putText(canvas, "TIP", (projected_x + 10, projected_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 0, 255), 1)
        
        # Putting the target circle on the canvas and logging the predicted point on the image in px
        cv2.circle(canvas, (avg_cx, avg_cy), 12, (0, 0, 255), 2)
        cv2.circle(canvas, (avg_cx, avg_cy), 4, (0, 0, 255), -1)
        inv_scale = 1 / avg_scale
        cv2.putText(canvas, f"TARGET CENTER: {inv_scale:.2f} pixel/cm", (avg_cx + 15, avg_cy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        camera_logger.info("%.3f, %.3f", avg_cx, avg_cy)

    print(f"Received {num_tags} tags. Center: ({avg_cx if num_tags > 0 else 0}, {avg_cy if num_tags > 0 else 0})")

if __name__ == "__main__":
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Connecting to {MQTT_BROKER}...")
    client.connect(MQTT_BROKER, 1883, 60)
    client.loop_start()

    while True:
        cv2.imshow("AprilTag Real-Time Map", canvas)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    client.loop_stop()