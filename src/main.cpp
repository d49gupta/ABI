#include "april_tags.hpp"
#include "mqtt.hpp"
#include "pencil.hpp"

int main()
{
    Publisher publisher;
    // AprilTagDetector detector(7.5, 29); Dharama 
    // 2.5 = distance from edge of tag to the center of the tag
    // 6 = distance from the center of tag to the center of the circle
    AprilTagDetector detector(2.5, 5); // Evan Test page
    GT2 pencil(30);

    int width = 640;
    int height = 480;
    int size = width * height;
    unsigned char* buffer = new unsigned char[size];

    while (std::cin.read((char*)buffer, size)) 
    {
        image_u8_t img = { .width = width, .height = height, .stride = width, .buf = buffer };
        detector.detectTags(&img);
        std::string jsonOutput = detector.JSONOutput();
        publisher.sendMessage("camera/detections", jsonOutput);
        std::cin.ignore(size / 2);

        pencil.readRaw();
        int reading = pencil.getLatestReading();
        publisher.sendMessage("pencil/reading", std::to_string(reading));
    }
    
    return 0;
}
