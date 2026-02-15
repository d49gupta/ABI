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
    
    # Configure for a fast preview stream
    config = picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)})
    picam2.configure(config)
    picam2.start()

    # Make sure these match your physical checkerboard (internal corners only!)
    INNER_ROWS = 10 
    INNER_COLS = 17

    print("--- Calibration Mode ---")
    print("Press 'c' to capture current frame")
    print("Press 'q' to finish and calculate")

    try:
        while True:
            # capture_array() in a loop gives us a live feed
            frame = picam2.capture_array()
            
            # Picamera2 RGB888 is actually RGB, OpenCV likes BGR
            show = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord("c"):
                found, corners = calib.add_frame(show, INNER_ROWS, INNER_COLS)
                if found:
                    print(f"Accepted frame #{len(calib.obj_points)}")
                    cv2.drawChessboardCorners(show, (INNER_COLS, INNER_ROWS), corners, found)
                    cv2.imshow("Calibration", show)
                    cv2.waitKey(500) 

            cv2.imshow("Calibration", show)
            
            if key == ord("q"):
                break
                
    finally:
        if len(calib.obj_points) > 0:
            print("\nCalculating calibration... please wait.")
            h, w = frame.shape[:2]
            ret, K, dist = calib.calibrate((w, h))
            calib.save()
        
        picam2.stop()
        cv2.destroyAllWindows()