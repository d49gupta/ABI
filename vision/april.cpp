#include <iostream>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

int main() {
    // 1. Setup AprilTag Detector
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    // 2. Input Pipeline: Capture from Camera
    std::string pipeline_in = "libcamerasrc ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true";
    cv::VideoCapture cap(pipeline_in, cv::CAP_GSTREAMER);

    // 3. Output Pipeline: Stream to Laptop (Use your Laptop IP)
    // We add 'tune=zerolatency' and 'intra-refresh=true' to kill the lag
    std::string pipeline_out = "appsrc ! videoconvert ! x264enc tune=zerolatency intra-refresh=true bitrate=2000 ! rtph264pay ! udpsink host=192.168.0.34 port=5000";
    cv::VideoWriter writer(pipeline_out, cv::CAP_GSTREAMER, 0, 30, cv::Size(640, 480));

    if (!cap.isOpened() || !writer.isOpened()) {
        std::cerr << "Error opening pipelines!" << std::endl;
        return -1;
    }

    cv::Mat frame, gray;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Convert OpenCV Mat to AprilTag image format
        image_u8_t img = { .width = gray.cols, .height = gray.rows, .stride = gray.cols, .buf = gray.data };

        zarray_t *detections = apriltag_detector_detect(td, &img);

        // Draw bounding boxes on the original color frame
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            for (int j = 0; j < 4; j++) {
                cv::line(frame, cv::Point(det->p[j][0], det->p[j][1]),
                         cv::Point(det->p[(j+1)%4][0], det->p[(j+1)%4][1]),
                         cv::Scalar(0, 255, 0), 2);
            }
            cv::putText(frame, std::to_string(det->id), cv::Point(det->c[0], det->c[1]),
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }

        // Send the frame with drawings to the network
        writer.write(frame);

        apriltag_detections_destroy(detections);
    }

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    return 0;
}
