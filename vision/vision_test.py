import cv2
import numpy as np

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

def generate_36h11_tag(dictionary):
    # NOTE: The generated tag is too ideal to be picked up by the following code
    # Generate a valid april tag (36h11)
    # Generate a tag with ID 0
    tag_id = 0
    tag_size = 400  # pixels
    tag_image = cv2.aruco.generateImageMarker(dictionary, tag_id, tag_size)

    # Save the generated tag
    cv2.imwrite(r"C:\Users\evanh\ABI\vision\april_tag.png", tag_image)
    print("Generated april_tag.png!")

# Load image
image = cv2.imread(r"C:\Users\evanh\ABI\vision\april_tag_ex.png")
if image is None:
    raise FileNotFoundError("Could not load 'april_tag.png'. Check path and file integrity.")

# Convert to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Initialize the AprilTag detector using aruco
# Available families: TAG16h5, TAG25h9, TAG36h11, etc.
parameters = cv2.aruco.DetectorParameters()
# dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# Detect tags
corners, ids, rejected = detector.detectMarkers(gray)

# Draw detections and print pose info
if ids is not None:
    for i, corner in enumerate(corners):
        # Draw marker outline
        cv2.polylines(image, [corner.astype(int)], True, (0, 255, 0), 2)

        # Compute the center
        cX = int(corner[0][:, 0].mean())
        cY = int(corner[0][:, 1].mean())
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

        print(f"Tag ID: {ids[i][0]}, Center: ({cX}, {cY})")

    cv2.imshow("AprilTag Detection", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("No AprilTags detected.")

# Camera calibration parameters (you need to calibrate your camera or use estimates)
# These are example values - replace with your actual camera parameters
focal_length = image.shape[1]  # Rough estimate
center = (image.shape[1] / 2, image.shape[0] / 2)
camera_matrix = np.array([
    [focal_length, 0, center[0]],
    [0, focal_length, center[1]],
    [0, 0, 1]
], dtype=float)
dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

# Real-world size of the AprilTag in meters (measure your printed tag)
marker_size = 0.05  # 5cm - CHANGE THIS to your actual tag size

if ids is not None:
    for i, corner in enumerate(corners):
        # Draw marker outline
        cv2.polylines(image, [corner.astype(int)], True, (0, 255, 0), 2)

        # Define 3D points of the marker corners in object space
        obj_points = np.array([
            [-marker_size/2,  marker_size/2, 0],
            [ marker_size/2,  marker_size/2, 0],
            [ marker_size/2, -marker_size/2, 0],
            [-marker_size/2, -marker_size/2, 0]
        ], dtype=np.float32)

        # Get 2D corner points
        img_points = corner[0]

        # Solve PnP to get rotation and translation vectors
        success, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)

        if success:
            # Draw axis
            cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, marker_size/2)

            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            # Extract x, y position and yaw (rotation around z-axis)
            x = tvec[0][0]
            y = tvec[1][0]
            z = tvec[2][0]

            # Calculate yaw from rotation matrix
            # Yaw is the rotation around the Z-axis
            yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
            yaw_degrees = np.degrees(yaw)

            print(f"\nTag ID: {ids[i][0]}")
            print(f"Position (x, y, z): ({x:.3f}m, {y:.3f}m, {z:.3f}m)")
            print(f"Yaw: {yaw_degrees:.2f}°")
            print(f"Rotation Matrix:\n{rotation_matrix}")
            print(f"Translation Vector:\n{tvec}")

            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            # Create 4x4 transformation matrix
            transformation_matrix = np.eye(4)
            transformation_matrix[:3, :3] = rotation_matrix  # Top-left 3x3: rotation
            transformation_matrix[:3, 3] = tvec.flatten()     # Top-right 3x1: translation

            print("Transformation Matrix:")
            print(transformation_matrix)

    cv2.imshow("AprilTag Detection with Pose", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("No AprilTags detected.")
    print(f"Number of rejected candidates: {len(rejected)}")