import cv2
import numpy as np
from picamera2 import Picamera2

# 1. Initialize
picam2 = Picamera2()

# 2. Configure for Module 3 (imx708)
# We use RGB888 here as it's standard for color analysis
config = picam2.create_still_configuration(main={"format": "RGB888", "size": (640, 480)})
picam2.configure(config)
picam2.start()

print("Camera active. Capturing frame...")

# 3. Capture directly into a NumPy array
frame = picam2.capture_array()

# 4. Manual "Color Analysis" (The OpenCV way)
# This replaces the old ColorAnalyzer
avg_color_per_row = np.average(frame, axis=0)
avg_color = np.average(avg_color_per_row, axis=0)
print(f"Average Color (RGB): {avg_color}")

# 5. Save the image to verify
# Note: OpenCV expects BGR, so we swap colors before saving
cv2.imwrite("analysis_test.jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

picam2.stop()
print("Done! Image saved to analysis_test.jpg")
