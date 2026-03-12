import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import time
from robot.globals import *
from robot.sensors import *
import robot.main as main

# 1. Configuration
buffer_size = 100
update_interval = 20 

# Use deques for everything to keep memory constant
tx_data = deque(maxlen=buffer_size)
dx_data = deque(maxlen=buffer_size)
dy_data = deque(maxlen=buffer_size)
dz_data = deque(maxlen=buffer_size)
tpencil_data = deque(maxlen=buffer_size)
pencil_data = deque(maxlen=buffer_size)

# 3. Setup Figure
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6, 5))
plt.subplots_adjust(hspace=0.4)

# Configure Top Plot
ax1.set_title("Camera State")
ax1.grid(True, alpha=0.3)
line_dx, = ax1.plot([], [], label='dx', color='#ff4b4b')
line_dy, = ax1.plot([], [], label='dy', color='#2ecc71')
line_dz, = ax1.plot([], [], label='dz', color="#291fbd")
ax1.legend(loc='upper right', ncol=3)
ax1.set_ylim(-100, 100)

# Configure Bottom Plot
ax2.set_ylim(-1, 8)
ax2.set_title("Pencil State")
ax2.grid(True, alpha=0.3)
line_pencil, = ax2.plot([], [], label='pencil depth', color='#3498db')
ax2.legend(loc='upper right')

def update(frame):
    if not correction_buffer or not pencil_buffer:
        print("No available data to plot")
        return line_dx, line_dy, line_dz, line_pencil
    
    # Get latest data points
    correction = correction_buffer[-1]
    pencil = pencil_buffer[-1]

    # Append to our local plotting deques
    tx_data.append(correction.timestamp)
    dx_data.append(correction.dx)
    dy_data.append(correction.dy)
    dz_data.append(correction.dz)
    tpencil_data.append(pencil.timestamp)
    pencil_data.append(pencil.distance)

    # Update line data
    line_dx.set_data(tx_data, dx_data)
    line_dy.set_data(tx_data, dy_data)
    line_dz.set_data(tx_data, dz_data)
    line_pencil.set_data(tpencil_data, pencil_data)

    # 4. Handle Sliding Window (show last 5 seconds)
    if tx_data and tpencil_data:
        latest_now = max(tx_data[-1], tpencil_data[-1])
        ax1.set_xlim(latest_now - 5, latest_now)
        ax2.set_xlim(latest_now - 5, latest_now)

    return line_dx, line_dy, line_dz, line_pencil

if __name__ == "__main__":
    connect_sensors()
    start_sensors()
    time.sleep(2)

    if not connection_status():
        print("Failed to read sensors")
        stop_sensors()
        exit(1)

    ani = FuncAnimation(fig, update, interval=update_interval, blit=False, cache_frame_data=False)
    plt.show()

    stop_sensors()