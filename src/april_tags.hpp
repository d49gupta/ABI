#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <unordered_map>
struct AprilTag
{
    int id;
    double x;
    double y;
    double center_x;
    double center_y;
};

struct Point2D
{
    double x;
    double y;
};

class AprilTagDetector 
{
public:
    AprilTagDetector(int tag_size, int offset) : tag_size(tag_size), offset(offset)
    {
        tag_offset = offset / (1.0 * tag_size);
        tf = tag36h11_create();
        td = apriltag_detector_create();
        apriltag_detector_add_family(td, tf);

        tag_positions[0] = {-tag_offset, -tag_offset};
        tag_positions[1] = {tag_offset, -tag_offset};
        tag_positions[2] = {-tag_offset,  tag_offset};
        tag_positions[3] = {tag_offset,  tag_offset};
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
    int tag_size; // in cm
    int offset; // in cm
    float tag_offset;
    std::unordered_map<int, Point2D> tag_positions;
};
