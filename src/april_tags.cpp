#include "april_tags.hpp"

// Main detection entry point
// Passes raw greyscale frame to the AprilTag libray, then iterates over every detection.
// For each tag detected_tags[i] = [tagID, px centroid, projected center point, pixel-to-mm scaling factor, estimated Z depth]
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
        float curr_tag_size = this->tag_sizes[det->id];
        td.scale = curr_tag_size * 2 / std::sqrt(dx*dx + dy*dy); // mm / pixels
        td.est_z = FOCAL_LENGTH * curr_tag_size * 2 / std::sqrt(dx*dx + dy*dy); // mm
        
        this->detected_tags.push_back(td);
    }

    apriltag_detections_destroy(detections);
    return this->num_tags > 0;
}

// Takes tag's homography matrix and projexcts a point at a known physical offset in tag-local coords into px space
// Use it to find an approximate of the physical center of the calibration target
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

// Getter returning number of tags deteceted in last frame
size_t AprilTagDetector::detectionCount() 
{
    return num_tags;
}

// Serialize all detected tags into a JSON string containing the count and an array of tag objects 
// to publish to camera/detections MQTT topic
std::string AprilTagDetector::JSONOutput()
{
    std::stringstream ss;

    // filter out the 3-point calibration tags
    std::vector<AprilTag> tags_4pt;
    for (const auto& tag : detected_tags)
        if (tag.id != 5 && tag.id != 6)
            tags_4pt.push_back(tag);

    ss << "{ \"count\": " << tags_4pt.size() << ", \"tags\": [";

    for (size_t i = 0; i < tags_4pt.size(); i++) {
        ss << "{"
        << "\"id\":"       << tags_4pt[i].id << ","
        << "\"x\":"        << tags_4pt[i].x  << ","
        << "\"y\":"        << tags_4pt[i].y  << ","
        << "\"center_x\":" << tags_4pt[i].center_x << ","
        << "\"center_y\":" << tags_4pt[i].center_y << ","
        << "\"scale\":"  << tags_4pt[i].scale << ","
        << "\"est_z\":"  << tags_4pt[i].est_z << "}";
        
        if (i < tags_4pt.size() - 1) ss << ",";
    }

    ss << "]}";
    return ss.str();
}

// Serialize all detected tags into a JSON string containing the count and an array of tag objects 
// to publish to camera/detections MQTT topic for 3-point calibration
std::string AprilTagDetector::JSONOutput3pt()
{
    std::stringstream ss;
    // filter out the 4-point calibration tags
    std::vector<AprilTag> tags_3pt;
    for (const auto& tag : detected_tags)
        if (tag.id == 4 || tag.id == 5 || tag.id == 6)
            tags_3pt.push_back(tag);

    ss << "{ \"count\": " << tags_3pt.size() << ", \"tags\": [";
    for (size_t i = 0; i < tags_3pt.size(); i++) {
        ss << "{"
           << "\"id\":"       << tags_3pt[i].id << ","
           << "\"x\":"        << tags_3pt[i].x  << ","
           << "\"y\":"        << tags_3pt[i].y  << ","
           << "\"center_x\":" << tags_3pt[i].center_x << ","
           << "\"center_y\":" << tags_3pt[i].center_y << ","
           << "\"scale\":"    << tags_3pt[i].scale << ","
           << "\"est_z\":"    << tags_3pt[i].est_z << "}";
        if (i < tags_3pt.size() - 1) ss << ",";
    }
    ss << "]}";
    return ss.str();
}
