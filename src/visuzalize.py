import paho.mqtt.client as mqtt
import json
import cv2
import numpy as np

# --- CONFIGURATION ---
MQTT_BROKER = "192.168.0.40"
MQTT_TOPIC = "detections"
WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480

canvas = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH, 3), dtype=np.uint8)

def on_connect(client, userdata, flags, rc):
    print(f"Connected to Pi with result code {rc}")
    client.subscribe(MQTT_TOPIC)

def on_message(client, userdata, msg):
    global canvas
    try:
        payload = msg.payload.decode("utf-8")
        data = json.loads(payload)
        canvas.fill(0) 

        tags = data.get("tags", [])
        for tag in tags:
            x = int(tag["x"])
            y = int(tag["y"])
            tag_id = tag["id"]

            cv2.circle(canvas, (x, y), 8, (0, 255, 0), -1)
            cv2.putText(canvas, f"ID: {tag_id}", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        print(f"Received {data['count']} tags")

    except Exception as e:
        print(f"Error parsing data: {e}")

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