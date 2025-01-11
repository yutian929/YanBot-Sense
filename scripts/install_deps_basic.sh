#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

# basic
## common
sudo apt update
check_success
sudo apt upgrade
check_success
sudo apt install python3-pip git
check_success
sudo apt install ros-${ROS_DISTRO}-image-transport
check_success
sudo apt install ros-${ROS_DISTRO}-rgbd-launch
check_success
sudo apt install ros-${ROS_DISTRO}-ddynamic-reconfigure
check_success
## camera
sudo apt install libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev 
### Build librealsense
mkdir -p thirdparties/
cd thirdparties/

if [ -d "YanBot-Sense_librealsense" ]; then
    echo "YanBot-Sense_librealsense directory already exists. Skipping clone."
else
    echo "Cloning YanBot-Sense_librealsense..."
    git clone https://github.com/yutian929/YanBot-Sense_librealsense.git YanBot-Sense_librealsense
    check_success
fi

cd YanBot-Sense_librealsense
./scripts/setup_udev_rules.sh
check_success

if [ -d "build" ]; then
    echo "Removing existing YanBot-Sense_librealsense build directory..."
    rm -rf build
fi
mkdir build && cd build
cmake ..
check_success
make -j$(( $(nproc) / 2 ))
check_success
sudo make install
check_success
cd ../../..
### create realsense-ros locally
mkdir -p src/camera/
cd src/camera/
if [ -d "YanBot-Sense_realsense_ros" ]; then
    echo "YanBot-Sense_realsense_ros directory already exists. Skipping clone."
else
    echo "Cloning YanBot-Sense_realsense_ros..."
    git clone https://github.com/yutian929/YanBot-Sense_realsense_ros.git YanBot-Sense_realsense_ros
    check_success
fi
cd ../../