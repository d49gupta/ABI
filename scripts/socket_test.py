import socket
import sys
import time
import signal

def signal_handler(sig, frame):
    print("\n[SIGINT] Clean exit initiated. Closing socket and stopping script...")
    sys.exit(0)

def run_robot_loop():
    host = '127.0.0.1' 
    port = 5000
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((host, port))
            print("Connected to Robot!")

            while True:
                command = "10,0,0"
                s.sendall(command.encode('utf-8'))
                
                data = s.recv(1024).decode('utf-8')
                print(f"Robot moved. Current: {data}")
                
                time.sleep(1)

        except KeyboardInterrupt:
            print("\nStopping...")
        except Exception as e:
            print(f"Connection lost: {e}")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    run_robot_loop()