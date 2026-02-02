import socket
import egm_pb2
import signal
import sys
import traceback
import numpy as np

# Configuration
UDP_IP = "0.0.0.0" 
UDP_PORT = 6510    
sequence_number = 0

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
steps = 100
total_displacement = 100.0
increment = total_displacement / steps
current_z = 0.0
initial_pos = None

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
            pos = None
            orient = None

            if robot_message.HasField('feedBack'):
                joints=robot_message.feedBack.joints.joints
                joint_angles=np.array(list(joints))
            if robot_message.HasField('rapidExecState'):
                rapid_running = robot_message.rapidExecState.state == robot_message.rapidExecState.RAPID_RUNNING
            if robot_message.HasField('motorState'):
                motors_on = robot_message.motorState.state == robot_message.motorState.MOTORS_ON
            if robot_message.feedBack.HasField('cartesian'):
                pos = robot_message.feedBack.cartesian.pos
                cartesian_pos = np.array([pos.x, pos.y, pos.z])
                orient = robot_message.feedBack.cartesian.orient
                quaternion = np.array([orient.u0, orient.u1, orient.u2, orient.u3])
            
            if not motors_on:
                break

            if initial_pos is None and pos is not None:
                initial_pos = cartesian_pos
            
            relative_pos = cartesian_pos - initial_pos if initial_pos is not None else None

            print(f"Received EGM Robot message: SeqNo={robot_message.header.seqno}, Joints={joint_angles}, RapidRunning={rapid_running}, MotorsOn={motors_on}")
            print(f"Relative Pose: XYZ={relative_pos}")
            
            if abs(relative_pos[2]) >= total_displacement:
                print("Reached total displacement. Stopping.")
                break
                
            current_z += increment
            sensor_message = egm_pb2.EgmSensor()
            sensor_message.header.mtype=egm_pb2.EgmHeader.MessageType.Value('MSGTYPE_CORRECTION')
            sensor_message.header.seqno = sequence_number
            sequence_number += 1
            sensor_message.planned.cartesian.pos.x = 0.0
            sensor_message.planned.cartesian.pos.y = 0.0
            sensor_message.planned.cartesian.pos.z = current_z
            sensor_message.planned.cartesian.orient.u0 = 1.0
            sensor_message.planned.cartesian.orient.u1 = 0.0
            sensor_message.planned.cartesian.orient.u2 = 0.0
            sensor_message.planned.cartesian.orient.u3 = 0.0

            sensor_data = sensor_message.SerializeToString()
            sock.sendto(sensor_data, addr)            

        except socket.timeout:
            continue

except Exception as e:
    print(f"An error occurred: {e}")
    traceback.print_exc()

finally:
    print(sequence_number)
    print("Closing socket.")
    sock.close()
    sys.exit(0)