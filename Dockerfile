# Use the official ROS 2 Humble base image
FROM ros:humble

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install core tools and dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    build-essential \
    # ros-humble-rviz2 \
    # ros-humble-gazebo-ros-pkgs \
    # ros-humble-rosbag2 \
    # ros-humble-rosbag2-storage \
    # ros-humble-rqt-plot \
    # ros-humble-foxglove-bridge \
    raspi-config \
    ros-humble-cv-bridge \
    # libopencv-dev \
    python3-opencv \
    # libx11-xcb1 \
    # libxcb-icccm4 \
    # libxcb-image0 \
    # libxcb-keysyms1 \
    # libxcb-randr0 \
    # libxcb-render0 \
    # libxcb-shape0 \
    # libxcb-shm0 \
    # libxcb-sync1 \
    # libxcb-util1 \
    # libxcb-xfixes0 \
    # libxkbcommon-x11-0 \
    # qtbase5-dev \
    # qtchooser \
    # qt5-qmake \
    # qtbase5-dev-tools \
    # x11-apps \
    # x11-utils \
    i2c-tools \
    libi2c-dev \
    libgpiod-dev \
    nano \
    libxml2-utils \
    # gdb \
    && rm -rf /var/lib/apt/lists/*

# Set up a ROS workspace
RUN mkdir -p /home/ros2_ws/src
WORKDIR /home/ros2_ws

# Install demo ROS 2 nodes
RUN apt-get update && apt-get install -y \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py

# Create and activate Python virtual environment + install Python deps
RUN python3 -m venv /root/ros2_venv && \
    /root/ros2_venv/bin/pip install --upgrade pip && \
    /root/ros2_venv/bin/pip install \
        setuptools==58.2.0 \
        pyyaml \
        empy==3.3.4 \
        catkin_pkg \
        lark-parser \
        numpy

# Install rosbags/bagpy (use venv pip)
# RUN /root/ros2_venv/bin/pip install rosbags bagpy

# Auto-source ROS and venv for interactive sessions
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_venv/bin/activate" >> ~/.bashrc && \
    echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

# Build workspace once to initialize it (optional)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_venv/bin/activate && colcon build || true"

# Default shell
CMD ["/bin/bash"]
