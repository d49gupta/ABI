import numpy as np
import cv2
from picamera2 import Picamera2
from calibration import Calibration 
import time

picam2 = Picamera2()
config = picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)})
picam2.configure(config)
picam2.start()

calib = Calibration()
INNER_ROWS = 10
INNER_COLS = 17

last_capture_time = time.time()
capture_interval = 3.0

print("Video active. 'c' to capture, 'q' to finish/calibrate.")

try:
    while True:
        frame = picam2.capture_array()
        current_time = time.time()

        if current_time - last_capture_time > capture_interval:
            bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            print("Processing calibration frame...")
            found, corners = calib.add_frame(bgr_frame, INNER_ROWS, INNER_COLS)

            if found:
                print(f"Accepted frame #{len(calib.obj_points)}")

            else:
                print("Board not found...")
        
            last_capture_time = current_time
            
        time.sleep(0.01)
        if len(calib.obj_points) >= 20: 
            print("Target reached. Starting calibration...")
            break

except Exception as e:
    print(f"Error: {e}")

finally:
    if len(calib.obj_points) > 0:
        print("Calibrating... please wait.")
        ret, K, dist = calib.calibrate(frame.shape[:2][::-1])
        calib.save()
        print("Calibration saved.")
    
    cv2.destroyAllWindows()
    picam2.stop()