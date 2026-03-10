UWaterloo Group 25 Mechatronics Engineering Capstone Group with ABI Ltd
Automated calibration for 6-DOF ABB Robot arm on conveyor belt.
**Dharma Gupta, Evan Hall, James Li, Justin Joseph**

# ABB Robot End-Effector Calibration System 

This project runs on a Raspberry Pi 5 mounted to a custom end-effector attached to an ABB robotic arm. It performs 4-point (and in-progress 3-point) spatial calibration using AprilTag fiducial markers and a contact pencil sensor, publishing all sensor data over MQTT for consumption by the robot's calibration logic.

---

## File Descriptions

### `main.cpp`
The application entry point that orchestrates the two concurrent sensor streams. It initializes the AprilTag detector, MQTT publisher, and pencil sensor, then spawns a dedicated pencil-reading thread while the main thread continuously processes camera frames piped in via `stdin`. A `SIGINT` handler ensures both threads shut down cleanly on Ctrl+C.

### `april_tags.cpp` / `april_tags.hpp`
Handles AprilTag detection and spatial projection from camera frames. For each detected tag, it computes the raw pixel centroid, projects a known physical offset point through the tag's homography matrix to locate the calibration center in pixel space, and estimates the tag's Z depth and pixel-to-mm scale factor. Results are serialized to JSON and published to the `camera/detections` MQTT topic.

### `mqtt.cpp` / `mqtt.hpp`
A lightweight MQTT publisher wrapper built on the Mosquitto library. It connects to a local broker on startup and exposes a `sendMessage()` method used by both sensor streams to publish JSON payloads to their respective topics (`camera/detections` and `pencil/reading`).

### `pencil.cpp` / `pencil.hpp`
Manages the GT2 analog contact/distance sensor connected via I2C ADC. It configures the ADC on startup, continuously reads raw 16-bit samples, converts them to millimeters via linear interpolation, and sets a contact flag when the measured distance exceeds a defined threshold. Readings are stored in a circular cache and serialized to JSON for MQTT publishing.

---

## MQTT Topics

| Topic | Source | Description |
|---|---|---|
| `camera/detections` | `april_tags.cpp` | JSON array of detected AprilTags with position, scale, and estimated depth |
| `pencil/reading` | `pencil.cpp` | JSON object with raw ADC value, distance in mm, and contact flag |

---

## Dependencies

- [AprilTag](https://github.com/AprilRobotics/apriltag) — fiducial marker detection
- [Mosquitto](https://mosquitto.org/) (`libmosquitto`) — MQTT client library
- Linux I2C (`/dev/i2c-1`) — ADC communication for pencil sensor

