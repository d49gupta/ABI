import pandas as pd
import paho.mqtt.client as mqtt
import time
import json
import threading
import signal

# --- CONFIG ---
BROKER_ADDRESS = "127.0.0.1"
PORT = 1883
CAMERA_CSV = "C:/Users/dharm/OneDrive - University of Waterloo/Documents/School/Fourth Year/Capstone/ABI/test_logs/camera_logs.csv"
PENCIL_CSV = "C:/Users/dharm/OneDrive - University of Waterloo/Documents/School/Fourth Year/Capstone/ABI/test_logs/pencil_logs.csv"

PENCIL_TOPIC = "pencil/reading"
CAMERA_TOPIC = "camera/detections"

PENCIL_HZ = 100
CAMERA_HZ = 10

mqtt_lock = threading.Lock()
running = True

def signal_handler(sig, frame):
    global running
    print("\n[!] SIGINT detected. Closing threads...")
    running = False

signal.signal(signal.SIGINT, signal_handler)

def pencil_worker(client, df):
    df.columns = df.columns.str.strip()
    print(f"Pencil Thread: Started ({PENCIL_HZ}Hz)")
    
    for index, row in df.iterrows():
        if not running:
            break

        payload = json.dumps({
            "ms": int(row["ms"]),
            "millimeters": float(row["dist"]),
        })
        print(f"Pencil Thread publishing")
        
        with mqtt_lock:
            client.publish(PENCIL_TOPIC, payload)
            
        time.sleep(1 / PENCIL_HZ)
    print("Pencil Thread: Finished.")

def camera_worker(client, df):
    df.columns = df.columns.str.strip()
    print(f"Camera Thread: Started ({CAMERA_HZ}Hz)")
    
    for index, row in df.iterrows():
        if not running:
            break

        payload = json.dumps({
            "ms": int(row["ms"]),
            "correction_x": int(row["correction_x"]),
            "correction_y": int(row["correction_y"])
        })
        print(f"Camera Thread publishing")
        
        with mqtt_lock:
            client.publish(CAMERA_TOPIC, payload)
            
        time.sleep(1 / CAMERA_HZ)
    print("Camera Thread: Finished.")

def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, "simulator")
    client.connect(BROKER_ADDRESS, PORT)

    print("Starting simulation")
    try:
        pencil_df = pd.read_csv(PENCIL_CSV)
        camera_df = pd.read_csv(CAMERA_CSV)
    except Exception as e:
        print(f"Error loading CSVs: {e}")
        return

    t_pencil = threading.Thread(target=pencil_worker, args=(client, pencil_df), daemon=True)
    t_camera = threading.Thread(target=camera_worker, args=(client, camera_df), daemon=True)

    t_pencil.start()
    t_camera.start()

    while running and (t_pencil.is_alive() or t_camera.is_alive()):
            time.sleep(0.1)

    client.disconnect()
    print("All playback complete.")

if __name__ == "__main__":
    main()