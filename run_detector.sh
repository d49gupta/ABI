#!/bin/bash

# 1. Handle the build directory
if [ ! -d "build" ]; then
  echo "Creating build directory..."
  mkdir build
fi

cd build || exit

# 2. Run CMake and Make
# Assuming you use CMake; if just a Makefile, remove the cmake line
cmake ..
make -j$(nproc)

# 3. Check if build was successful before running
if [ -f "/home/sourdough/ABI/build/tag_detector" ]; then
    echo "Starting camera stream..."
    # 4. Execute the pipeline
    # We use -o - to send video to stdout, which | sends to your app's stdin
    rpicam-vid -t 0 --inline --framerate 20 --width 640 --height 480 --codec yuv420 -o - 2> camera_log.txt | /home/sourdough/ABI/build/tag_detector
else
    echo "Build failed. Check errors above."
    exit 1
fi
