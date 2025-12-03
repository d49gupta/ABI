import cv2
import numpy as np

class CameraEsimate():
    def __init__(self, tag_size=1.0):
        parameters = cv2.aruco.DetectorParameters()
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        self.tag_size = tag_size
        self.camera_T_tag = {} # Transform from tag frame to camera frame
        fs = cv2.FileStorage("calibration.yaml", cv2.FILE_STORAGE_READ)
        self.K = fs.getNode("camera_matrix").mat()
        self.dist = fs.getNode("dist_coeffs").mat()
        fs.release()

        board_spacing = 0.15
        self.board_positions = {
                                1: np.array([-board_spacing / 2, -board_spacing / 2, 0.0]),  # top-left
                                0: np.array([ board_spacing / 2, -board_spacing / 2, 0.0]),  # top-right
                                3: np.array([-board_spacing / 2,  board_spacing / 2, 0.0]),  # bottom-left
                                2: np.array([ board_spacing / 2,  board_spacing / 2, 0.0]),  # bottom-right
                                }

        self.center_transform = None # Transform from center frame to camera frame
        self.center_px = None

    def detect_tags(self, frame, project=True):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        tags, ids, rejected = self.detector.detectMarkers(gray)
        if ids is None:
            print("No tags detected.")
            return False

        if project:
            cv2.aruco.drawDetectedMarkers(frame, tags, ids)

        half = self.tag_size / 2.0
        obj_pts = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
            ], dtype=np.float32)
        
        for corners, tag_id in zip(tags, ids):
            img_pts = corners.reshape(-1, 2).astype(np.float32)
            retval, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.K, self.dist)
            R, _ = cv2.Rodrigues(rvec)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3]  = tvec.reshape(3)
            self.camera_T_tag[int(tag_id)] = T

        return True
    
    def detect_center(self, frame, project=True):
        
        if len(self.camera_T_tag) == 0:
            return
        
        center_sum = np.zeros(3, dtype=np.float32)

        for tag_id, camera_T_tag in self.camera_T_tag.items():
            R = camera_T_tag[:3, :3]
            t = camera_T_tag[:3, 3]

            p_center_tag = self.board_positions[tag_id]
            p_center_cam = R @ p_center_tag + t
            center_sum += p_center_cam

        center_avg = center_sum / len(self.camera_T_tag)
        camera_T_center = np.eye(4, dtype=np.float32)
        camera_T_center[:3, 3] = center_avg
        self.camera_T_center = camera_T_center
        print(center_avg)

        if project:
            self.project_center(frame)

    def project_center(self, frame):
        x, y, z = self.camera_T_center[:3, 3]
        x_norm = x / z
        y_norm = y / z
        u = self.K[0, 0] * x_norm + self.K[0, 2]
        v = self.K[1, 1] * y_norm + self.K[1, 2]
        self.center_px = (int(u), int(v))
        cv2.circle(frame, camera.center_px, 5, (0, 0, 255), -1)

if __name__=="__main__":
    cap = cv2.VideoCapture(0)
    camera = CameraEsimate(0.031)

    if not cap.isOpened():
        print("Error: Cannot open camera")
        exit()

    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("Failed to grab frame")
            break

        camera.detect_tags(frame)
        camera.detect_center(frame)
        cv2.imshow("AprilTag Detection with Pose", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()