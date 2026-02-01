import scripts.egm_pb2 as egm_pb2
import socket
import signal
import sys
import numpy as np
from dataclasses import dataclass

# --- DATACLASS ---
@dataclass
class robotState:
    initial_pos : np.ndarray
    pos : np.ndarray
    quaternion : np.ndarray
class EGMState:
    udp_ip: str = "0.0.0.0"
    udp_port: int = 6510
    sequence_number: int = 0
    connected: bool = False
    egm_addr = None
    sock = None

# --- STATE ---
robot_state = robotState(
    initial_pos = None,
    pos = None,
    quaternion = None
)
egm_state = EGMState()

def connect_socket():
    egm_state.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    egm_state.sock.settimeout(1.0) 
    egm_state.sock.bind((egm_state.udp_ip, egm_state.udp_port))

def disconnect_socket():
    egm_state.sock.close()

def receive_data():
    try:
        data, addr = egm_state.sock.recvfrom(65536)
        if not egm_state.connected:
            print(f"CONNECTED: Receiving data from {addr}")
            egm_state.connected = True
            egm_state.egm_addr = addr

        robot_message = egm_pb2.EgmRobot()
        robot_message.ParseFromString(data)

        if robot_message.HasField('cartesian'):
            pos = robot_message.feedBack.cartesian.pos
            curr_pos = np.array([pos.x, pos.y, pos.z])
            orient = robot_message.feedBack.cartesian.orient
            robot_state.quaternion = np.array([orient.u0, orient.u1, orient.u2, orient.u3])
            robot_state.initial_pos = curr_pos if robot_state.initial_pos is None else robot_state.initial_pos

        robot_state.pos = curr_pos - robot_state.initial_pos if robot_state.initial_pos is not None else curr_pos
            
    except socket.timeout:
        return None
    
def send_cartesian_command(x, y, z, u0, u1, u2, u3):
    command_message = egm_pb2.EgmSensor()
    command_message.header.seqno = egm_state.sequence_number
    command_message.header.mtype = egm_pb2.EgmHeader.MessageType.Value('MSGTYPE_CORRECTION')

    command_message.planned.cartesian.pos.x = x
    command_message.planned.cartesian.pos.y = y
    command_message.planned.cartesian.pos.z = z

    command_message.planned.cartesian.orient.u0 = u0
    command_message.planned.cartesian.orient.u1 = u1
    command_message.planned.cartesian.orient.u2 = u2
    command_message.planned.cartesian.orient.u3 = u3

    egm_state.sequence_number += 1
    sensor_data = command_message.SerializeToString()

    egm_state.sock.sendto(sensor_data, egm_state.egm_addr)