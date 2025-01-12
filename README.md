# YanBot-Sense
YanBot-Sense: Sense Part of YanBot. A ROS-based indoor robot system for 3D semantic mapping and autonomous navigation, leveraging RGB-D cameras and LiDAR to identify and navigate to objects in dynamic environments.
# Prepare
## Basic Prepare
```bash
bash install_deps_basic.sh
pip install -r requirements.txt
cd YanBot-Sense/
catkin_make
source devel/setup.bash
```
## SLAM Prepare
### [RTAB-MAP](http://wiki.ros.org/rtabmap_ros)
```bash
bash scripts/install_deps_rtabmap.sh
```
### [ORB-SLAM3](https://github.com/thien94/orb_slam3_ros/tree/master)
```bash
bash scripts/install_deps_orbslam3.sh
```
## Data Prepare
### [roboTHOR/AI2THOR](https://ai2thor.allenai.org/robothor/documentation/)
```txt
Nothing to do. Included in requirements.txt
```
### [TUM](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download#)
```bash
Visit [TUM]("https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download#"), and download the rosbag you need.
```

# Use (Sorted by algorithms)
## RTAB-MAP + roboTHOR/TUM
```bash
# slam
## RGB-D Handheld Mapping
roslaunch rtabmap_launch rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false

# data
## roboTHOR
rosrun robothor_pkg robothor_node.py
## realsense d435
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

## ORB-SLAM3 + TUM
```bash
# slam
roslaunch orb_slam3_ros tum_rgbd.launch

# data
rosbag play rgbd_dataset_freiburg1_xyz.bag
```

# Perception(TODO)
