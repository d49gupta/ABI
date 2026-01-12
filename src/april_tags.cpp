#include "april_tags.hpp"

bool AprilTagDetector::detectTags(image_u8_t* img) 
{
    zarray_t* detections = apriltag_detector_detect(this->td, img);
    this->num_tags = zarray_size(detections);
    std::cout<<"Number of tags detected: " << this->num_tags << std::endl;
    this->detected_tags.clear();

    for (int i = 0; i < this->num_tags; i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        this->detected_tags.push_back({det->id, det->c[0], det->c[1]});
    }

    apriltag_detections_destroy(detections);
    return this->num_tags > 0;
}

size_t AprilTagDetector::detectionCount() 
{
    return num_tags;
}

std::string AprilTagDetector::JSONOutput()
{
    std::stringstream ss;
    ss << "{ \"count\": " << current_tags.size() << ", \"tags\": [";

    for (size_t i = 0; i < current_tags.size(); i++) {
        ss << "{\"id\":" << current_tags[i].id 
        << ",\"x\":" << current_tags[i].x 
        << ",\"y\":" << current_tags[i].y << "}";
        
        if (i < current_tags.size() - 1) ss << ",";
    }
    ss << "]}";
    return ss.str();
}