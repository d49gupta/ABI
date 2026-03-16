import paho.mqtt.client as mqtt

BROKER_IP = "192.168.0.54"
TOPIC = "robot/pi/cmd"
MESSAGE = "STOP"

def send_stop_signal():
    client = mqtt.Client()
    
    try:
        print(f"Connecting to broker at {BROKER_IP}...")
        client.connect(BROKER_IP, 1883)
        
        # Publish the message
        result = client.publish(TOPIC, MESSAGE)
        
        # Ensure the message actually sent before disconnecting
        result.wait_for_publish()
        print(f"Successfully sent '{MESSAGE}' to {TOPIC}")
        
    except Exception as e:
        print(f"Failed to send stop signal: {e}")
    finally:
        client.disconnect()

if __name__ == "__main__":
    send_stop_signal()