import cv2
import numpy as np
from picamera2 import Picamera2

class Calibration:
    def __init__(self):
        self.obj_points = []
        self.img_points = []
        self.K = None
        self.dist = None

    def add_frame(self, image, inner_rows, inner_cols, square_size=1.0):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Note: inner_rows/cols are the number of internal corners
        objp = np.zeros((inner_rows * inner_cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:inner_cols, 0:inner_rows].T.reshape(-1, 2)
        objp *= square_size

        ret, corners = cv2.findChessboardCorners(gray, (inner_cols, inner_rows), None)

        if not ret:
            print("No corners found. Try adjusting lighting or distance.")
            return False, None

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

        self.obj_points.append(objp)
        self.img_points.append(corners2)

        return True, corners2

    def calibrate(self, image_size):
        if len(self.obj_points) < 3:
            print("Need at least 3 frames to calibrate!")
            return False, None, None
            
        ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points,
            self.img_points,
            image_size,
            None,
            None
        )
        self.K, self.dist = K, dist
        print(f"Calibration RMS Error: {ret:.4f}")
        return ret, K, dist
    
    def save(self):
        if self.K is None: return
        fs = cv2.FileStorage("calibration_pi.yaml", cv2.FILE_STORAGE_WRITE)
        fs.write("camera_matrix", self.K)
        fs.write("dist_coeffs", self.dist)
        fs.release()
        print("Saved calibration to calibration_pi.yaml")

if __name__ == "__main__":
    calib = Calibration()
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)})
    picam2.configure(config)
    picam2.start()

    print("--- Headless Calibration Mode ---")
    print("Commands:")
    print("  [Enter] : Capture current frame")
    print("  q [Enter]: Finish and calculate")

    try:
        while True:
            user_input = input("Waiting (Enter=Capture, q=Finish): ").lower()
            
            if user_input == 'q':
                break
            
            frame = picam2.capture_array()
            process_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            found, corners = calib.add_frame(process_frame, 10, 17)
            if found:
                print(f"SUCCESS: Found corners! Total frames: {len(calib.obj_points)}")
                # cv2.imwrite(f"last_captured_{len(calib.obj_points)}.jpg", process_frame)
            else:
                print("FAILED: No corners detected. Reposition and try again.")
                # cv2.imwrite("failed_capture.jpg", process_frame)

    finally:
        if len(calib.obj_points) >= 3:
            print("\nCalculating calibration...")
            ret, K, dist = calib.calibrate((640, 480))
            calib.save()
        else:
            print("Not enough frames captured.")
        
        picam2.stop()