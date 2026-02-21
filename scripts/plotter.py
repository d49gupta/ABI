import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import random
from robot.globals import *
from robot.sensors import *

# 1. Configuration
buffer_size = 100
update_interval = 20 

# 2. Initialize Deques for all 4 variables
# x_buffer = deque([0]*buffer_size, maxlen=buffer_size)
# y_buffer = deque([0]*buffer_size, maxlen=buffer_size)
# z_buffer = deque([0]*buffer_size, maxlen=buffer_size)
a_buffer = deque([0]*buffer_size, maxlen=buffer_size) # New variable
time_index = list(range(buffer_size))

# 3. Setup Figure with 2 Subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
plt.subplots_adjust(hspace=0.4) # Add space between plots

# Configure Top Plot
ax1.set_ylim(-100, 100)
ax1.set_xlim(0, max(len(correction_buffer), len(pencil_buffer)) - 1)
ax1.set_title("Sensor Readings")
ax1.grid(True, alpha=0.3)
line_dx, = ax1.plot([], [], label='dx', color='#ff4b4b')
line_dy, = ax1.plot([], [], label='dy', color='#2ecc71')
line_dz, = ax1.plot([], [], label='dz', color='#3498db')
ax1.legend(loc='upper right', ncol=3)

# Configure Bottom Plot
ax2.set_ylim(0, 500)
ax2.set_xlim(0, buffer_size - 1)
ax2.set_title("Robot State")
ax2.grid(True, alpha=0.3)
line_a, = ax2.plot(time_index, a_buffer, label='state', color='#9b59b6', lw=2)
ax2.legend(loc='upper right')

def update(frame):
    if not correction_buffer or not pencil_buffer:
        return
    
    dx, dy, _, tx = zip(*correction_buffer)
    tz, dist = pencil_buffer.timestamp, pencil_buffer.distance
    _, dist, _, tz = random.uniform(200, 400)
    new_a = random.uniform(200, 400)

    line_dx.set_data(tx, dx)
    line_dy.set_data(tx, dy)
    line_dz.set_data(tz, dist)
    a_buffer.append(new_a)
    
    line_a.set_ydata(a_buffer)
    all_last_times = [t[-1] for t in [tx, tz] if len(t) > 0]
    
    if all_last_times:
        latest_now = max(all_last_times)
        # Show the last 5000ms (5 seconds) of data
        ax1.set_xlim(latest_now - 5000, latest_now)

    return line_dx, line_dy, line_dz, line_a

if __name__ == "__main__":
    connect_sensors()
    start_sensors()
    time.sleep(5)

    ani = FuncAnimation(fig, update, interval=update_interval, blit=True, cache_frame_data=False)
    plt.show()

    stop_sensors()
