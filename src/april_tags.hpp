#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <vector>

struct AprilTag
{
    int id;
    double center_x;
    double center_y;
}

class AprilTagDetector 
{
public:
    AprilTagDetector() {}

    ~AprilTagDetector() 
    {
        apriltag_detector_destroy(td);
        tag36h11_destroy(tf);
    }

    bool detectTags(image_u8_t* img);
    size_t detectionCount();
    std::string AprilTagDetector::JSONOutput();

    size_t num_tags;
    vector<AprilTag> detected_tags;

private:
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
};