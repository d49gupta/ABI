import cv2
import numpy as np

parameters = cv2.aruco.DetectorParameters()
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

def detect_tags(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags, ids, rejected = detector.detectMarkers(gray)
    if ids is None:
        print("No tags detected.")
        return None, 0

    cv2.aruco.drawDetectedMarkers(frame, tags, ids)
    print(f"Detected Tags: {ids.flatten()}")

    tag_center = []
    for tag in tags:
        curr_tag_center_x = 0
        curr_tag_center_y = 0

        for corner in tag[0]:
            curr_tag_center_x += corner[0]
            curr_tag_center_y += corner[1]
        
        curr_tag_center_x /= len(ids)
        curr_tag_center_y /= len(ids)
        tag_center.append((curr_tag_center_x, curr_tag_center_y))

    return tag_center, len(ids)

def find_jig_center(tag_center, numb_tags):
    jig_center_x = 0
    jig_center_y = 0
    for tag in tag_center:
        jig_center_x += tag[0]
        jig_center_y += tag[1]

    jig_center_x /= numb_tags
    jig_center_y /= numb_tags
    cx = int(jig_center_x)
    cy = int(jig_center_y)

    return cx, cy

if __name__=="__main__":
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("Error: Cannot open camera")
        exit()

    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("Failed to grab frame")
            break

        tag_center, numb_tags = detect_tags(frame)

        cx, cy = 0, 0
        if tag_center and numb_tags > 0:
            cx, cy = find_jig_center(tag_center, numb_tags)
        else:
            print("No april tags detected")

        cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)
        resized_img = cv2.resize(frame, (640, 640))
        cv2.imshow("Detected Tags + Center", resized_img)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()