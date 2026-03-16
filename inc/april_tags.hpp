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

#define FOCAL_LENGTH 292.3
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
    AprilTagDetector(float tag_size_corners, float tag_size_center, float offset) 
    : tag_size_corners(tag_size_corners), tag_size_center(tag_size_center), offset(offset)
    {
        tag_offset = offset;
        float two_tag_offset = 2.0 * tag_offset;
	    tf = tag36h11_create();
        td = apriltag_detector_create();
        apriltag_detector_add_family(td, tf);

        tag_positions[0] = {tag_offset, -tag_offset};
        tag_positions[1] = {-tag_offset, -tag_offset};
        tag_positions[2] = {tag_offset,  tag_offset};
        tag_positions[3] = {-tag_offset,  tag_offset};
	    tag_positions[4] = {0, 0};

        tag_positions_x[1] = {-two_tag_offset, -0};
        tag_positions_x[2] = {0, two_tag_offset};
        tag_positions_x[3] = {-two_tag_offset,  two_tag_offset};
        tag_positions_x[4] = {-two_tag_offset,  two_tag_offset};
	    tag_positions_x[0] = {0, 0};

        tag_positions_y[0] = {0, -two_tag_offset};
        tag_positions_y[1] = {-two_tag_offset, -two_tag_offset};
        tag_positions_y[2] = {0,  0};
        tag_positions_y[3] = {-two_tag_offset,  0};
	    tag_positions_y[4] = {-two_tag_offset, -two_tag_offset};

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
    std::string JSONOutput(const std::vector<AprilTag>& tags);
    std::string JSONOutputCenter();
    std::string JSONOutputX();
    std::string JSONOutputY();
    Point2D project_relative_point(apriltag_detection_t *det, double offset_x, double offset_y);

    size_t num_tags;
    std::vector<AprilTag> detected_tags;
    std::vector<AprilTag> detected_tags_x;
    std::vector<AprilTag> detected_tags_y;
private:
    apriltag_family_t *tf;
    apriltag_detector_t *td;
    float tag_size_corners; // in mm
    float tag_size_center; // in mm
    float offset; // in terms of tag_size_corners
    float tag_offset;
    std::unordered_map<int, Point2D> tag_positions; // ID: 4
    std::unordered_map<int, Point2D> tag_positions_x; // ID: 0
    std::unordered_map<int, Point2D> tag_positions_y; // ID: 2
    std::unordered_map<int, float> tag_sizes;
};

#endif
