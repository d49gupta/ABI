import socket
import egm_pb2
import signal
import sys
import traceback
import numpy as np

# Configuration
UDP_IP = "0.0.0.0" 
UDP_PORT = 6510    

keep_running = True
connected = False

def signal_handler(sig, frame):
    global keep_running
    print("\n[SIGINT] Shutting down...")
    keep_running = False

signal.signal(signal.SIGINT, signal_handler)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0) 
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for EGM data on port {UDP_PORT}...")
sequence_number = 0

try:
    while keep_running:
        try:
            data, addr = sock.recvfrom(65536) 
            
            if not connected:
                print(f"CONNECTED: Receiving data from {addr}")
                connected = True
            
            robot_message = egm_pb2.EgmRobot()
            robot_message.ParseFromString(data)

            joint_angles=None
            rapid_running=False
            motors_on=False

            if robot_message.HasField('feedBack'):
                joints=robot_message.feedBack.joints.joints
                joint_angles=np.array(list(joints))
            if robot_message.HasField('rapidExecState'):
                rapid_running = robot_message.rapidExecState.state == robot_message.rapidExecState.RAPID_RUNNING
            if robot_message.HasField('motorState'):
                motors_on = robot_message.motorState.state == robot_message.motorState.MOTORS_ON

            print(f"Received EGM Robot message: SeqNo={robot_message.header.seqno}, Joints={joint_angles}, RapidRunning={rapid_running}, MotorsOn={motors_on}")
            
            sensor_message = egm_pb2.EgmSensor()
            sensor_message.header.mtype = egm_pb2.EgmHeader.MessageType.MSGTYPE_CORRECTION
            sensor_message.header.seqno = sequence_number
            sequence_number += 1
            sensor_message.planned.joints.joints.extend([0, 0, 0, 0, 0, 0])
            serialized_msg = sensor_message.SerializeToString()
            sock.sendto(serialized_msg, addr)
            

        except socket.timeout:
            continue

except Exception as e:
    print(f"An error occurred: {e}")
    traceback.print_exc()

finally:
    print("Closing socket.")
    sock.close()
    sys.exit(0)