import paho.mqtt.client as mqtt
import json
import cv2
import numpy as np

# --- CONFIGURATION ---
MQTT_BROKER = "2607:fea8:1d66:2700::2d21" # Evan's Connect
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
            center_x = int(tag["center_x"])
            center_y = int(tag["center_y"])
            tag_id = tag["id"]

            cv2.circle(canvas, (x, y), 8, (0, 255, 0), -1)
            cv2.putText(canvas, f"ID: {tag_id}", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 255, 255), 1)
            
            cv2.circle(canvas, (center_x, center_y), 5, (255, 255, 0), -1)
            cv2.putText(canvas, f"ID: {tag_id}", (center_x + 10, center_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 255, 255), 1)

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