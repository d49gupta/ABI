#include "april_tags.hpp"

bool AprilTagDetector::detectTags(image_u8_t* img) 
{
    zarray_t* detections = apriltag_detector_detect(this->td, img);
    this->num_tags = zarray_size(detections);
    std::cout<<"Number of tags detected: " << this->num_tags << std::endl;
    this->detected_tags.clear();

    for (size_t i = 0; i < this->num_tags; i++) 
    {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        AprilTag td;

        td.id = det->id;
        td.x = det->c[0];
        td.y = det->c[1];

        Point2D p = project_relative_point(det, tag_positions[det->id].x, tag_positions[det->id].y);
        td.center_x = p.x;
        td.center_y = p.y;
        
        double dx = det->p[1][0] - det->p[0][0];
        double dy = det->p[1][1] - det->p[0][1];
        td.scale = this->tag_size / std::sqrt(dx*dx + dy*dy);
        
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
    
    pixel.x = (x_prime / z_prime);
    pixel.y = (y_prime / z_prime);

    // Apply radial distortion correction
    const double IMAGE_WIDTH = 640.0;   // Adjust to your camera resolution
    const double IMAGE_HEIGHT = 480.0;  // Adjust to your camera resolution
    const double cx = IMAGE_WIDTH / 2.0;
    const double cy = IMAGE_HEIGHT / 2.0;
    
    // Vector from image center to projected point
    double dx = pixel.x - cx;
    double dy = pixel.y - cy;
    
    // Distance from center
    double r = std::sqrt(dx * dx + dy * dy);
    
    // Radial correction factor (tune this value experimentally)
    const double DISTORTION_K = -0.25;  // Start with 0.05, adjust as needed
    double correction = r * DISTORTION_K;
    
    // Apply correction along radial direction
    //if (r > 0) {
    if (false) {
        pixel.x += (dx / r) * correction;  // Use + for barrel distortion
        pixel.y += (dy / r) * correction;  // Use - for pincushion distortion
    }


    return pixel;
}

size_t AprilTagDetector::detectionCount() 
{
    return num_tags;
}

std::string AprilTagDetector::JSONOutput()
{
    std::stringstream ss;
    ss << "{ \"count\": " << detected_tags.size() << ", \"tags\": [";

    for (size_t i = 0; i < detected_tags.size(); i++) {
        ss << "{"
        << "\"id\":"       << detected_tags[i].id << ","
        << "\"x\":"        << detected_tags[i].x  << ","
        << "\"y\":"        << detected_tags[i].y  << ","
        << "\"center_x\":" << detected_tags[i].center_x << ","
        << "\"center_y\":" << detected_tags[i].center_y << ","
        << "\"scale\":"  << detected_tags[i].scale <<
        << "}";
        
        if (i < detected_tags.size() - 1) ss << ",";
    }

    ss << "]}";
    return ss.str();
}
