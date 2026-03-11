#include "april_tags.hpp"
#include "mqtt.hpp"
#include "pencil.hpp"
#include <thread>
#include <atomic>
#include <chrono>
#include <csignal>
#include <vector>
#include <iostream>

std::atomic<bool> keepRunning(true);

// Gracefully signals all threads and loops to stop when Ctrl + C
void signalHandler(int signum)
{
    std::cout << "\n[System] CTRL+C detected. Shutting down threads..." << std::endl;
    keepRunning = false;
}

// Reads raw data from the pencil sensor, serialize to JSON and publishes to pencil/reading MQTT topic.
void runPencilThread(GT2* pencil, Publisher* publisher) 
{
    while (keepRunning) 
    {
        pencil->readRaw();
        std::string jsonReading = pencil->JSONOutput();
        publisher->sendMessage("pencil/reading", jsonReading);        
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // ~100 Hz loop
    }
}

// Spawns the pencil thread, then continuously reads raw greyscale images from stdin
// Wraps each frame into image_u8_t struct, runs AprilTag detection, serialized results to JSON and publishes to camera/detections
void runCameraLoop(AprilTagDetector& detector, Publisher& publisher, uint8_t* buffer, int size, int width, int height, GT2& pencil) 
{
    std::thread pThread(runPencilThread, &pencil, &publisher);
    while (keepRunning && std::cin.read((char*)buffer, size)) 
    {
	    std::cout<<"Reading Camera Frames"<<std::endl;
        image_u8_t img = { .width = width, .height = height, .stride = width, .buf = buffer };
        detector.detectTags(&img);
        std::string jsonOutput = detector.JSONOutput();
        std::string jsonOutput3pt = detector.JSONOutput3pt();
        publisher.sendMessage("camera/detections", jsonOutput);
        publisher.sendMessage("camera/3pt_calibration", jsonOutput3pt);
        std::cin.ignore(size / 2); 
    }

    keepRunning = false;
    if (pThread.joinable())
        pThread.join();
}

// Sets up the signal handler, instatiates the Publisher, AprilTagDetector (set sizes), and GTS pencil sensor
// Launches runCameraLoop
int main(int argc, char** argv) 
{
    std::signal(SIGINT, signalHandler);

    Publisher publisher;
    // pass in radius of corner tags, center tag, estimated offset in tag radius units
    // float tag_size_corners, float tag_size_center, float tag_size_side, float offset, float side_offset
    AprilTagDetector detector(22.5, 11.5, 17, 2.375, 6.7);	    
    GT2 pencil(30);

    int width = 640;
    int height = 480;
    int size = width * height; 
    std::vector<uint8_t> buffer(size);

    try {
        runCameraLoop(detector, publisher, buffer.data(), size, width, height, pencil);
    } catch (const std::exception& e) {
        std::cerr << "Runtime Error: " << e.what() << std::endl;
    }

    return 0;
}
