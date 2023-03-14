## Dependencies
```sudo apt install python3-catkin-tools```

## Create catkin ws
```bash
mkdir -p ~/catkin_ws/src
cd catkin_ws
catkin init
catkin config --extend /opt/ros/noetic ## or /opt/ros/melodic depending on the version

# clone 
cd src
git clone https://github.com/daBaret/orbslam3_wrapper_ros.git

# modify CMakeList by putting the correct path to the ORB_SLAM3 installation
vim orb-slam3-wrapper/CMakeLists.txt 
# Modify this part
# set(ORB_SLAM3_DIR
#    $ENV{HOME}/ros/orbslam3_ws/orb-slam3
#)

# Build the package
catkin build orb_slam3_wrapper

# Extract volabulary
tar -xf orb-slam3-wrapper/config/ORBvoc.txt.tar.gz -C orb-slam3-wrapper/config/

# Source
source ../devel/setup.bash

# Launch
roslaunch orb_slam3_wrapper mono.launch
```
