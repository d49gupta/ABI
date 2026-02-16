#ifndef APRIL_TAGS_HPP
#define APRIL_TAGS_HPP

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <cmath>

#define FOCAL_LENGTH 292.3 // TODO: Pull from calibration_arducam.yaml
struct AprilTag
{
    int id;
    double x;
    double y;
    double center_x;
    double center_y;
    double est_z;
    double scale;
};

struct Point2D
{
    double x;
    double y;
};

class AprilTagDetector 
{
public:
    AprilTagDetector(int tag_size_corners, int tag_size_center, int offset) 
    : tag_size_corners(tag_size_corners), tag_size_center(tag_size_center), offset(offset)
    {
       // tag_offset = offset / (1.0 * tag_size_corners);
        tag_offset = offset;
	tf = tag36h11_create();
        td = apriltag_detector_create();
        apriltag_detector_add_family(td, tf);

        tag_positions[0] = {tag_offset, -tag_offset};
        tag_positions[1] = {-tag_offset, -tag_offset};
        tag_positions[2] = {tag_offset,  tag_offset};
        tag_positions[3] = {-tag_offset,  tag_offset};
	    tag_positions[4] = {0, 0};

        tag_sizes[0] = tag_size_corners;
        tag_sizes[1] = tag_size_corners;
        tag_sizes[2] = tag_size_corners;
        tag_sizes[3] = tag_size_corners;
        tag_sizes[4] = tag_size_center;
    }

    ~AprilTagDetector() 
    {
        apriltag_detector_destroy(td);
        tag36h11_destroy(tf);
    }

    bool detectTags(image_u8_t* img);
    size_t detectionCount();
    std::string JSONOutput();
    Point2D project_relative_point(apriltag_detection_t *det, double offset_x, double offset_y);

    size_t num_tags;
    std::vector<AprilTag> detected_tags;

private:
    apriltag_family_t *tf;
    apriltag_detector_t *td;
    int tag_size_corners; // in cm
    int tag_size_center; // in cm
    int offset; // in terms of tag_size_corners
    float tag_offset;
    std::unordered_map<int, Point2D> tag_positions;
    std::unordered_map<int, float> tag_sizes;
};

#endif
