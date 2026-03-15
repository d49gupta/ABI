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
    AprilTagDetector(float tag_size_corners, float tag_size_center, float tag_size_side, float offset, float side_offset) 
    : tag_size_corners(tag_size_corners), tag_size_center(tag_size_center), tag_size_side(tag_size_side), tag_offset(offset), side_offset(side_offset)
    {
	    tf = tag36h11_create();
        td = apriltag_detector_create();
        apriltag_detector_add_family(td, tf);

        tag_positions[0] = {tag_offset, -tag_offset};
        tag_positions[1] = {-tag_offset, -tag_offset};
        tag_positions[2] = {tag_offset,  tag_offset};
        tag_positions[3] = {-tag_offset,  tag_offset};
	    tag_positions[4] = {0, 0};

        tag_positions_x[4] = {-side_offset, 0};
        tag_positions_x[5] = {0, 0};
        tag_positions_x[6] = {-side_offset, -side_offset}; // TODO: Check signs and that tag 5 is x axis tag

        tag_positions_y[4] = {0, -side_offset};
        tag_positions_y[5] = {-side_offset, -side_offset};
        tag_positions_y[6] = {0, 0}; // TODO: Check signs and that tag 6 is y axis tag

        tag_sizes[0] = tag_size_corners;
        tag_sizes[1] = tag_size_corners;
        tag_sizes[2] = tag_size_corners;
        tag_sizes[3] = tag_size_corners;
        tag_sizes[4] = tag_size_center;
        tag_sizes[5] = tag_size_side;
        tag_sizes[6] = tag_size_side;
    }

    ~AprilTagDetector() 
    {
        apriltag_detector_destroy(td);
        tag36h11_destroy(tf);
    }

    bool detectTags(image_u8_t* img);
    size_t detectionCount();
    std::string JSONOutput();
    std::string JSONOutput3pt();
    Point2D project_relative_point(apriltag_detection_t *det, double offset_x, double offset_y);

    size_t num_tags;
    std::vector<AprilTag> detected_tags;
    std::vector<AprilTag> detected_tags_x;
    std::vector<AprilTag> detected_tags_y;

private:
    apriltag_family_t *tf;
    apriltag_detector_t *td;
    float tag_size_corners; // half the width of tag in mm
    float tag_size_center; // half the width of tag in mm
    float tag_size_side; // half the width of tag in mm
    float side_offset; // center to center offset / half width of tag 
    float tag_offset; // in terms of tag_size_corners (mm)
    std::unordered_map<int, Point2D> tag_positions;
    std::unordered_map<int, Point2D> tag_positions_x;
    std::unordered_map<int, Point2D> tag_positions_y;
    std::unordered_map<int, float> tag_sizes;
};

#endif
