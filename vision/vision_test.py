import cv2
import numpy as np

# INPUT IMAGE PATH - CHANGE TO YOUR IMAGE
image_path = r"C:\Users\evanh\ABI\vision\april_tags.jpg"

# LOAD IMAGE
image = cv2.imread(image_path)
if image is None:
    raise FileNotFoundError(f"Could not load {image_path}. Check path.")

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# APRILTAG DETECTOR (Aruco wrapper)
parameters = cv2.aruco.DetectorParameters()
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# Detect AprilTags
corners, ids, rejected = detector.detectMarkers(gray)
if ids is None:
    print("No tags detected.")
    exit()

cv2.aruco.drawDetectedMarkers(image, corners, ids)
print(f"Detected Tags: {ids.flatten()}")

# CAMERA PARAMETERS - REPLACE WITH YOUR CALIBRATION
# focal_length = 3000 #15.12  # in pixels (Iphone 12 back camera)

# Estimate the focal length
focal_length_mm = 4.2  # actual physical focal length (not the 26mm "equivalent")
sensor_width_mm = 4.55  # iPhone 12 sensor width
image_width_pixels = image.shape[1]  # e.g., 3024 or 4032 pixels

focal_length_pixels = (4.2 / 4.55) * image_width_pixels

print(f"Calculated focal length: {focal_length_pixels:.2f} pixels")
print(f"Image size: {image.shape[1]} x {image.shape[0]}\n")

cx = image.shape[1] / 2
cy = image.shape[0] / 2
camera_matrix = np.array([
    [focal_length_pixels, 0, cx],
    [0, focal_length_pixels, cy],
    [0, 0, 1]
], dtype=np.float32)
dist_coeffs = np.zeros((4, 1), dtype=np.float32)  # change if you have distortion

# TAG SIZE (meters) 
marker_size = 0.05

# BOARD LAYOUT (meters)
board_spacing = 0.0709+0.05
# Tag ID mapping wrt to the board:
# top-right -> 3, top-left -> 2, bottom-right -> 1, bottom-left -> 0
# board_positions = {
#     2: np.array([-board_spacing / 2,  board_spacing / 2, 0.0]),  # top-left
#     3: np.array([ board_spacing / 2,  board_spacing / 2, 0.0]),  # top-right
#     0: np.array([-board_spacing / 2, -board_spacing / 2, 0.0]),  # bottom-left
#     1: np.array([ board_spacing / 2, -board_spacing / 2, 0.0]),  # bottom-right
# }
board_positions = {
    2: np.array([-board_spacing / 2, -board_spacing / 2, 0.0]),  # top-left
    3: np.array([ board_spacing / 2, -board_spacing / 2, 0.0]),  # top-right
    0: np.array([-board_spacing / 2,  board_spacing / 2, 0.0]),  # bottom-left
    1: np.array([ board_spacing / 2,  board_spacing / 2, 0.0]),  # bottom-right
}

# SOLVE PNP FOR EACH DETECTED TAG
transforms = {}  # tag_id -> 4x4 camera->tag transform
for i, corner in enumerate(corners):
    tag_id = int(ids[i][0])
    if tag_id not in board_positions:
        print(f"Tag {tag_id} detected but not part of layout. Skipping.")
        continue

    # object points: tag corners in tag-local frame (centered at origin)
    obj_pts = np.array([
        [-marker_size / 2,  marker_size / 2, 0.0],
        [ marker_size / 2,  marker_size / 2, 0.0],
        [ marker_size / 2, -marker_size / 2, 0.0],
        [-marker_size / 2, -marker_size / 2, 0.0],
    ], dtype=np.float32)

    img_pts = corner[0].astype(np.float32)

    success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, dist_coeffs)
    if not success:
        print(f"PnP failed for tag {tag_id}")
        continue

    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()

    transforms[tag_id] = T
    print(f"\nTag {tag_id} camera->tag transform:\n{T}")

# COMPUTE camera->board for each tag and collect estimates
camera_T_board_estimates = []
for tag_id, camera_T_tag in transforms.items():
    # Extract rotation and translation
    R = camera_T_tag[:3, :3]
    t = camera_T_tag[:3, 3]
    
    # Board origin in camera frame
    t_board = t - R @ board_positions[tag_id]
    
    # Construct camera->board transform
    camera_T_board = np.eye(4, dtype=np.float32)
    camera_T_board[:3, :3] = R
    camera_T_board[:3, 3] = t_board
    
    camera_T_board_estimates.append(camera_T_board)
    print(f"\nTag {tag_id} camera->board estimate:\n{camera_T_board}")

for i, T in enumerate(camera_T_board_estimates):
    print(f"\nEstimate {i}:\n{T}")

if len(camera_T_board_estimates) == 0:
    raise RuntimeError("No valid camera->board estimates were computed.")

# AVERAGE TRANSLATIONS
translations = np.array([T[:3, 3] for T in camera_T_board_estimates])
mean_translation = translations.mean(axis=0)

# AVERAGE ROTATIONS using SVD projection to SO(3)
rotation_matrices = [T[:3, :3] for T in camera_T_board_estimates]
R_sum = np.zeros((3, 3), dtype=np.float32)
for R in rotation_matrices:
    R_sum += R
U, _, Vt = np.linalg.svd(R_sum)
mean_rotation = U @ Vt
# Ensure proper rotation (determinant = +1)
if np.linalg.det(mean_rotation) < 0:
    U[:, -1] *= -1
    mean_rotation = U @ Vt

# FINAL transform camera->board
final_T = np.eye(4, dtype=np.float32)
final_T[:3, :3] = mean_rotation
final_T[:3, 3] = mean_translation

print("\nFinal Camera->Board Transform Estimate:")
print(final_T)

# OPTIONAL: draw the board center on the image
# project the board-origin (0,0,0) in board frame to image using final_T inverse
# compute board->camera by inverting final_T
# board_T_camera = np.linalg.inv(final_T)
# board_origin_in_camera = board_T_camera[:3, 3]
# OPTIONAL: draw the board center on the image
# The board origin in camera frame is just the translation of camera->board
board_origin_in_camera = final_T[:3, 3]

# Project to pixel coordinates
point_cam = board_origin_in_camera.reshape(3, 1)
proj = camera_matrix @ point_cam
px = int(proj[0, 0] / proj[2, 0])
py = int(proj[1, 0] / proj[2, 0])

cv2.circle(image, (px, py), 6, (0, 0, 255), -1)
cv2.putText(image, 'Board Center', (px + 8, py), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

# project to pixel coordinates
point_cam = board_origin_in_camera.reshape(3, 1)
proj = camera_matrix @ point_cam
proj = (proj / proj[2]).astype(int)
px, py = int(proj[0]), int(proj[1])
cv2.circle(image, (px, py), 6, (0, 0, 255), -1)
cv2.putText(image, 'Board Center', (px + 8, py), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)


scale = 0.5
display_image = cv2.resize(image, None, fx=0.5, fy=0.5)
cv2.imshow("AprilTag Detection with Pose", display_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
