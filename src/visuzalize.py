import paho.mqtt.client as mqtt
import json
import cv2
import numpy as np

# --- CONFIGURATION ---
MQTT_BROKER = "fe80::80ee:98fe:7fcb:95c3%16"
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
        num_tags = len(tags)

        sum_cx = 0
        sum_cy = 0

        for tag in tags:
            # Draw the actual Tag locations for reference
            x = int(tag["x"])
            y = int(tag["y"])
            tag_id = tag["id"]

            cv2.circle(canvas, (x, y), 8, (0, 255, 0), -1)
            cv2.putText(canvas, f"ID: {tag_id}", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Accumulate the estimated centers
            sum_cx += tag["center_x"]
            sum_cy += tag["center_y"]

        # Calculate and draw the Average Circle Center
        if num_tags > 0:
            avg_cx = int(sum_cx / num_tags)
            avg_cy = int(sum_cy / num_tags)

            # Draw a larger, bright target circle at the average position
            cv2.circle(canvas, (avg_cx, avg_cy), 12, (0, 0, 255), 2)  # Red outline
            cv2.circle(canvas, (avg_cx, avg_cy), 4, (0, 0, 255), -1)  # Red center dot
            cv2.putText(canvas, "TARGET CENTER", (avg_cx + 15, avg_cy + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        print(f"Received {num_tags} tags. Center: ({avg_cx if num_tags > 0 else 0}, {avg_cy if num_tags > 0 else 0})")

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