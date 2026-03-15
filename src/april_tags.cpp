#include "april_tags.hpp"

bool AprilTagDetector::detectTags(image_u8_t* img) 
{
    zarray_t* detections = apriltag_detector_detect(this->td, img);
    this->num_tags = zarray_size(detections);
    std::cout<<"Number of tags detected: " << this->num_tags << std::endl;
    this->detected_tags.clear();

    #ifdef THREE_POINT
    this->detected_tags_x.clear();
    this->detected_tags_y.clear();
    #endif

    for (size_t i = 0; i < this->num_tags; i++) 
    {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        AprilTag td;

        td.id = det->id;
        td.x = det->c[0];
        td.y = det->c[1];

        const auto& pos = tag_positions[det->id];
        float curr_tag_size = this->tag_sizes[det->id];
        double dx = det->p[1][0] - det->p[0][0];
        double dy = det->p[1][1] - det->p[0][1];
        td.scale = curr_tag_size * 2 / std::sqrt(dx*dx + dy*dy); // mm / pixels
        td.est_z = FOCAL_LENGTH * curr_tag_size * 2 / std::sqrt(dx*dx + dy*dy); // mm

        Point2D center = project_relative_point(det, pos.x, pos.y);
        td.center_x = center.x;
        td.center_y = center.y;
    
        this->detected_tags.push_back(td);
        
        #ifdef THREE_POINT
        Point2D x_est = project_relative_point(det, tag_positions_x[det->id].x, tag_positions_x[det->id].y);
        Point2D y_est = project_relative_point(det, tag_positions_y[det->id].x, tag_positions_y[det->id].y);

        AprilTag td_extra = td;
        td_extra.center_x = x_est.x;
        td_extra.center_y = x_est.y;
        this->detected_tags_x.push_back(td_extra);

        td_extra.center_x = y_est.x;
        td_extra.center_y = y_est.y;
        this->detected_tags_y.push_back(td_extra);
        #endif
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
    
    pixel.x = (x_prime / z_prime);
    pixel.y = (y_prime / z_prime);

    return pixel;
}

size_t AprilTagDetector::detectionCount() 
{
    return num_tags;
}

std::string AprilTagDetector::JSONOutput(const std::vector<AprilTag>& tags)
{
    std::stringstream ss;
    ss << "{ \"count\": " << tags.size() << ", \"tags\": [";

    for (size_t i = 0; i < tags.size(); i++) {
        ss << "{"
        << "\"id\":"       << tags[i].id << ","
        << "\"x\":"        << tags[i].x  << ","
        << "\"y\":"        << tags[i].y  << ","
        << "\"center_x\":" << tags[i].center_x << ","
        << "\"center_y\":" << tags[i].center_y << ","
        << "\"scale\":"  << tags[i].scale << ","
        << "\"est_z\":"  << tags[i].est_z << "}";
        
        if (i < tags.size() - 1) ss << ",";
    }

    ss << "]}";
    return ss.str();
}

std::string AprilTagDetector::JSONOutputCenter() {
    return JSONOutput(this->detected_tags);
}

std::string AprilTagDetector::JSONOutputX() {
    return JSONOutput(this->detected_tags_x);
}

std::string AprilTagDetector::JSONOutputY() {
    return JSONOutput(this->detected_tags_y);
}
