#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    // Usage:
    //   ./take_photo [output.jpg] [width] [height]
    std::string out = (argc >= 2) ? argv[1] : "capture.jpg";
    int width  = (argc >= 3) ? std::stoi(argv[2]) : 1280;
    int height = (argc >= 4) ? std::stoi(argv[3]) : 720;

    // --timeout gives the camera time to start and settle AE/AWB
    // --nopreview is best for headless
    // If autofocus causes issues, remove "--autofocus"
    std::string cmd =
        "rpicam-still "
        "--nopreview "
        "--timeout 1000 "
        "--width " + std::to_string(width) + " "
        "--height " + std::to_string(height) + " "
        "-o " + out;

    std::cout << "Running: " << cmd << "\n";
    int rc = std::system(cmd.c_str());
    if (rc != 0) {
        std::cerr << "Capture failed (exit code " << rc << ").\n";
        std::cerr << "Try running the command directly to see the error.\n";
        return 1;
    }

    std::cout << "Saved: " << out << "\n";
    return 0;
}
