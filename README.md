# ABB Robot End-Effector Calibration System

This project runs on a Raspberry Pi 5 mounted to a custom end-effector attached to an ABB robotic arm. It performs 4-point (and in-progress 3-point) spatial calibration using AprilTag fiducial markers and a contact pencil sensor. Sensor data is published over MQTT and consumed by a host machine running visualization and robot control logic. EGM (Externally Guided Motion) is used to issue real-time corrections to the ABB controller over UDP.

---

## Useage

### 4-Point Calibration Procedure

Follow these steps to perform the automated 4-point calibration for the robotic arm.

---

#### 1. Controller & Hardware Preparation
*   **Module Verification:** Ensure the `socket_cntrl.mod` file is loaded onto the robot controller.
*   **Operating Mode:** Set the robot controller to **Manual Mode**.
*   **Initial Positioning:** Move the robot arm to the **Home J** position.
*   **Plate Setup:** Place the Calibration Plate on the end of the conveyor, centered as much as possible. 
    > **Note:** The end-effector camera must be able to see at least one AprilTag to initiate the sequence.

#### 2. Raspberry Pi Configuration (via SSH)
Perform these steps to start the vision detection system:

1.  **Hardware Check:** Ensure the end-effector is powered on. Wait for the Raspberry Pi status LED to turn **solid green**.
2.  **Network Connection:** 
    *   Ping the device to find the IP: `ping sourdough.local`
    *   If no response, wait a few seconds or power cycle the device.
3.  **Establish SSH Session:**
execute:
    ```bash
    ssh sourdough@<RASPBERRY-PI IP-ADDRESS>
    sourdough 
    ```
    *Note: sourdough is the password for the ssh.*
    
#### 3. Running Calibration
*   **Routine Selection:** Set the program pointer on the controller to the `Calibrate` routine.
*   **Launch Control Script:** On the cell computer, open a terminal and execute:
    ```bash
    cd ABI
    python -m robot.main
    ```
    *This will launch the visualizer and start the conveyor until an AprilTag is detected.*

#### 4. Live Pose Plots (Optional)
To view live plots of the robot's pose during the calibration process, open a new terminal on the cell computer and execute:
```bash
cd ABI
python -m scripts.plotter
```
---

## Repository Structure

```
├── C++ (Runs on Raspberry Pi 5 — End-Effector)
│   ├── main.cpp
│   ├── april_tags.cpp / april_tags.hpp
│   ├── mqtt.cpp / mqtt.hpp
│   └── pencil.cpp / pencil.hpp
│
└── Python (Runs on Host Machine)
    ├── visualize.py          # MQTT subscriber + live AprilTag canvas
    ├── calibration.py        # Camera intrinsic calibration via chessboard
    ├── plotter.py            # Live matplotlib plot of camera + pencil state
    ├── logger.py             # CSV logging utility
    ├── 3dplot.py             # 3D SVD line-of-best-fit visualizer for calibration points
    ├── arducam.py            # Arducam capture test and color analysis
    ├── i2c.py                # I2C ADC raw read test script
    ├── socket_test.py        # TCP socket test for robot command interface
    └── calibration_pi.yaml   # Saved camera intrinsic matrix and distortion coefficients
```

---

## C++ Module Descriptions (Raspberry Pi 5)

### `main.cpp`
The application entry point that orchestrates the two concurrent sensor streams. It initializes the AprilTag detector, MQTT publisher, and pencil sensor, then spawns a dedicated pencil-reading thread while the main thread continuously processes camera frames piped in via `stdin`. A `SIGINT` handler ensures both threads shut down cleanly on Ctrl+C.

### `april_tags.cpp` / `april_tags.hpp`
Handles AprilTag detection and spatial projection from camera frames. For each detected tag it computes the raw pixel centroid, projects a known physical offset through the tag's homography matrix to locate the calibration center in pixel space, and estimates the tag's Z depth and pixel-to-mm scale factor. Results are serialized to JSON and published to the `camera/detections` MQTT topic.

### `mqtt.cpp` / `mqtt.hpp`
A lightweight MQTT publisher wrapper built on the Mosquitto library. It connects to a local broker on startup and exposes a `sendMessage()` method used by both sensor streams to publish JSON payloads to their respective topics (`camera/detections` and `pencil/reading`).

### `pencil.cpp` / `pencil.hpp`
Manages the GT2 analog contact/distance sensor connected via I2C ADC. It configures the ADC on startup, continuously reads raw 16-bit samples, converts them to millimeters via linear interpolation, and sets a contact flag when the measured distance exceeds a defined threshold. Readings are stored in a circular cache and serialized to JSON for MQTT publishing.

---

## Python Script Descriptions

### `visualize.py`
Subscribes to both MQTT sensor topics (`camera/detections` and `pencil/reading`) and renders a live OpenCV canvas showing detected AprilTag positions, each tag's projected calibration center, and the estimated pencil tip position based on a fixed mechanical offset from the camera. All camera and pencil readings are logged to timestamped CSV files via `CSVLogger`.

### `calibration.py`
Performs camera intrinsic calibration using a chessboard pattern captured live from the Arducam via `Picamera2`. The user interactively captures frames in headless mode; once at least 3 valid chessboard detections are collected, OpenCV's `calibrateCamera` computes the intrinsic matrix `K` and distortion coefficients, which are saved to `calibration_pi.yaml`.

### `egm_test.py`
Implements a UDP EGM (Externally Guided Motion) feedback loop with the ABB robot controller. It listens for robot state messages (joint angles, Cartesian pose, motor/RAPID state) and incrementally sends Z-axis position corrections via `EgmSensor` Protobuf messages until a total displacement target is reached. Includes graceful shutdown via `SIGINT`.

### `egm_pb2.py`
Auto-generated Python Protobuf bindings compiled from ABB's `egm.proto` schema. Defines all EGM message types used for robot communication including `EgmRobot`, `EgmSensor`, `EgmFeedBack`, `EgmPlanned`, `EgmHeader`, and associated state enumerations. This file should not be edited manually.

### `plotter.py`
Renders a live two-panel matplotlib plot showing camera correction deltas (`dx`, `dy`, `dz`) and pencil sensor distance over a sliding 5-second time window. Reads from shared `correction_buffer` and `pencil_buffer` globals populated by the robot sensor module, with animation driven by `FuncAnimation` at a configurable update interval.

### `logger.py`
A reusable CSV logging utility wrapping Python's `logging` module. Each log entry includes a timestamp, milliseconds since start, source module, line number, log level, a sequential message counter, and the message body. Provides `info`, `warning`, `error`, and `debug` methods with automatic counter increment.

### `3dplot.py`
A diagnostic script for visualizing calibration point accuracy in 3D space. Given a set of hardcoded XYZ calibration points, it computes a line of best fit using SVD (principal component analysis), plots the points and line in a 3D matplotlib figure, and reports min, max, and mean perpendicular distance errors from the fitted line.

### `arducam.py`
A minimal camera test script for the Arducam Module 3 (`imx708`) using `Picamera2`. Captures a single RGB frame, computes the average color across the image, saves the result as a JPEG, and prints the average RGB values to the console. Used for verifying camera connectivity and basic image output.

### `i2c.py`
A standalone I2C diagnostic script for directly reading raw ADC values from the ADS1115 (or equivalent) at address `0x48` using `smbus2`. Configures the ADC for single-ended input on AIN0 with a 6.144V full-scale range and prints raw bit values and converted voltages at 10Hz. Used for verifying I2C bus wiring and sensor calibration independent of the C++ stack.

### `socket_test.py`
A minimal TCP socket client for testing a robot command interface running on `localhost:5000`. Repeatedly sends a hardcoded `"X:2"` movement command and prints the robot's response. Includes `SIGINT` handling for clean exit. Used for early-stage integration testing of the TCP robot control layer.

### `calibration_pi.yaml`
A saved OpenCV camera calibration file containing the 3×3 intrinsic matrix `K` and 5-coefficient radial/tangential distortion vector for the Raspberry Pi camera at 640×480 resolution. Generated by `calibration.py` and consumed by the AprilTag detection pipeline to correct for lens distortion.

---

## MQTT Topics

| Topic | Producer | Consumer | Description |
|---|---|---|---|
| `camera/detections` | `april_tags.cpp` | `visualize.py`, `plotter.py` | JSON array of detected AprilTags with centroid, projected center, scale, and estimated Z depth |
| `pencil/reading` | `pencil.cpp` | `visualize.py`, `plotter.py` | JSON object with raw ADC value, distance in mm, and contact flag |

---

## Data Flow

```
Raspberry Pi 5 (End-Effector)
├── Camera frames (stdin pipe) ──► april_tags.cpp ──► MQTT: camera/detections
└── GT2 Pencil Sensor (I2C ADC) ──► pencil.cpp    ──► MQTT: pencil/reading

Host Machine
├── visualize.py   ◄── MQTT ──► Live OpenCV canvas + CSV logs
├── plotter.py     ◄── shared buffers ──► Live matplotlib telemetry
└── egm_test.py    ◄──► ABB Controller (UDP/EGM) ──► Real-time Z correction
```

---

## Dependencies

### C++
- [AprilTag](https://github.com/AprilRobotics/apriltag) — fiducial marker detection
- [Mosquitto](https://mosquitto.org/) (`libmosquitto`) — MQTT client
- Linux I2C (`/dev/i2c-1`) — ADC hardware interface

### Python
- `opencv-python` — image processing, chessboard calibration, canvas rendering
- `picamera2` — Raspberry Pi camera interface
- `paho-mqtt` — MQTT subscriber client
- `smbus2` — I2C ADC communication
- `protobuf` — EGM message serialization (`egm_pb2.py`)
- `numpy` — numerical operations and SVD
- `matplotlib` — 3D calibration plots and live telemetry