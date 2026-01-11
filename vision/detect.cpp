#include <iostream>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

int main() {
    // 1. Setup AprilTag
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    int width = 640;
    int height = 480;
    int size = width * height;
    unsigned char* buffer = new unsigned char[size];

    while (std::cin.read((char*)buffer, size)) {
        // Create the image object from the raw buffer
        image_u8_t img = { .width = width, .height = height, .stride = width, .buf = buffer };

        zarray_t *detections = apriltag_detector_detect(td, &img);

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            
            // Just print the data - no drawing, no lag!
            std::cout << "Detected Tag ID: " << det->id 
                      << " at Center: (" << det->c[0] << ", " << det->c[1] << ")" << std::endl;
        }
        apriltag_detections_destroy(detections);
        
        // Skip the U and V color planes (for YUV420) to get to the next Y (grayscale) frame
        std::cin.ignore(size / 2);
    }
    return 0;
}
