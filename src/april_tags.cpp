#include "april_tags.hpp"

bool AprilTagDetector::detectTags(image_u8_t* img) 
{
    zarray_t* detections = apriltag_detector_detect(this->td, img);
    this->num_tags = zarray_size(detections);
    std::cout<<"Number of tags detected: " << this->num_tags << std::endl;
    this->detected_tags.clear();

    for (size_t i = 0; i < this->num_tags; i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        AprilTag td;

        td.id = det->id;
        td.x = det->c[0];
        td.y = det->c[1];

        Point2D p = project_relative_point(det, 0.0, 0.0);
        td.center_x = p.x;
        td.center_y = p.y;
        this->detected_tags.push_back(td);
    }

    apriltag_detections_destroy(detections);
    return this->num_tags > 0;
}

Point2D AprilTagDetector::project_relative_point(apriltag_detection_t *det, double offset_x, double offset_y) 
{
    double* h = det->H->data;

    double x_prime = h[0] * offset_x + h[1] * offset_y + h[2];
    double y_prime = h[3] * offset_x + h[4] * offset_y + h[5];
    double z_prime = h[6] * offset_x + h[7] * offset_y + h[8];

    Point2D pixel;
    pixel.x = x_prime / z_prime;
    pixel.y = y_prime / z_prime;

    return pixel;
}

size_t AprilTagDetector::detectionCount() 
{
    return num_tags;
}

std::string AprilTagDetector::JSONOutput()
{
    ss << "{ \"count\": " << detected_tags.size() << ", \"tags\": [";

    for (size_t i = 0; i < detected_tags.size(); i++) {
        ss << "{"
        << "\"id\":"       << detected_tags[i].id << ","
        << "\"x\":"        << detected_tags[i].x  << ","
        << "\"y\":"        << detected_tags[i].y  << ","
        << "\"center_x\":" << detected_tags[i].center_x << ","
        << "\"center_y\":" << detected_tags[i].center_y
        << "}";
        
        if (i < detected_tags.size() - 1) ss << ",";
    }

    ss << "]}";
    return ss.str();
}
