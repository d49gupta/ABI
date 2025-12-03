import cv2
import numpy as np

class Calibration:
    def __init__(self):
        self.obj_points = []
        self.img_points = []
        self.K = None
        self.dist = None

    def add_frame(self, image, inner_rows, inner_cols, square_size=1.0):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        objp = np.zeros((inner_rows * inner_cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:inner_cols, 0:inner_rows].T.reshape(-1, 2)
        objp *= square_size

        ret, corners = cv2.findChessboardCorners(gray, (inner_cols, inner_rows))

        if not ret:
            return False, None

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

        self.obj_points.append(objp)
        self.img_points.append(corners2)

        return True, corners2

    def calibrate(self, image_size):
        ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points,
            self.img_points,
            image_size,
            None,
            None
        )
        self.K, self.dist = K, dist
        print("Calibration Accuracy:", ret)
        return ret, K, dist
    
    def save(self):
        K = np.array(self.K)
        dist = np.array(self.dist)
        fs = cv2.FileStorage("calibration.yaml", cv2.FILE_STORAGE_WRITE)
        fs.write("camera_matrix", K)
        fs.write("dist_coeffs", dist)
        fs.release()

        print("Saved calibration to calibration.yaml")

if __name__ == "__main__":
    calib = Calibration()
    cap = cv2.VideoCapture(0)

    INNER_ROWS = 10
    INNER_COLS = 17

    print("Press 'c' to capture calibration frame")
    print("Press 'q' to finish and calibrate")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        show = frame.copy()
        key = cv2.waitKey(1) & 0xFF
        if key == ord("c"):
            print("Calibration frame requested")
            found, corners = calib.add_frame(frame, INNER_ROWS, INNER_COLS)

            if found:
                cv2.drawChessboardCorners(show, (INNER_COLS, INNER_ROWS), corners, found)
                print(f"Accepted frame #{len(calib.obj_points)}")

        cv2.imshow("Calibration", show)
        if key == ord("q"):
            break

    ret, K, dist = calib.calibrate(frame.shape[:2][::-1])
    calib.save()
    cap.release()
    cv2.destroyAllWindows()
