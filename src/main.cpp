#include "april_tags.hpp"
#include "mqtt.hpp"
#include "pencil.hpp"
#include "binary_pencil.hpp"
#include <thread>
#include <atomic>
#include <chrono>
#include <csignal>
#include <vector>
#include <iostream>

std::atomic<bool> keepRunning(true);

void signalHandler(int) 
{
    std::cout << "\n[System] CTRL+C detected. Shutting down threads..." << std::endl;
    keepRunning = false;
}

void runPencilThread(GT2* pencil, Publisher* publisher) 
{
    while (keepRunning) 
    {
        pencil->readRaw();
        std::string jsonReading = pencil->JSONOutput();
        publisher->sendMessage("pencil/reading", jsonReading);        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void runBinaryPencilThread(P10DLB* binaryPencil, Publisher* publisher) 
{
    while (keepRunning) 
    {
        std::string jsonOutput = binaryPencil->JSONOutput();
        publisher->sendMessage("binary_pencil/reading", jsonOutput);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void runCameraLoop(AprilTagDetector& detector, Publisher& publisher, uint8_t* buffer, int size, int width, int height, P10DLB& binaryPencil) 
{
    std::thread bpThread(runBinaryPencilThread, &binaryPencil, &publisher);
    while (keepRunning && std::cin.read((char*)buffer, size)) 
    {
	    std::cout<<"Reading Camera Frames"<<std::endl;
        image_u8_t img = { .width = width, .height = height, .stride = width, .buf = buffer };
        detector.detectTags(&img);
        std::string jsonOutput = detector.JSONOutput();
        publisher.sendMessage("camera/detections", jsonOutput);
        std::cin.ignore(size / 2); 
    }

    keepRunning = false;
    if (bpThread.joinable())
        bpThread.join();
}

int main() 
{
    std::signal(SIGINT, signalHandler);

    Publisher publisher;
    // pass in radius of corner tags, center tag, estimated offset in tag radius units
    AprilTagDetector detector(22.5, 11.5, 2.375);	    
    P10DLB binaryPencil;

    int width = 640;
    int height = 480;
    int size = width * height; 
    std::vector<uint8_t> buffer(size);

    try {
        runCameraLoop(detector, publisher, buffer.data(), size, width, height, binaryPencil);
    } catch (const std::exception& e) {
        std::cerr << "Runtime Error: " << e.what() << std::endl;
    }

    return 0;
}
