# YanBot-Sense

Sense Part. Camrea, Lidar... sensors data publisher. Robot's perception of the surrounding environment  including 3D semantic map generation.

## Basic

Basic prerequisites for running subsequent programs. Need to be prepared first.

```bash
# YanBot-Sense/
bash scripts/install_deps_basic.sh
pip install -r requirements.txt
catkin_make
source devel/setup.bash
```

## sense

Comprehensive node, including launch file.

```bash
roslaunch sense_pkg XXX.launch
```

## camera

Camera(rsD435) ROS support.

### [realsense adapted](https://github.com/yutian929/YanBot-Sense_realsense_ros.git)

```bash
bash scripts/install_deps_camera_realsense.sh
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

## SLAM

SLAM Module.

### [RTAB-MAP](http://wiki.ros.org/rtabmap_ros)
```bash
bash scripts/install_deps_rtabmap.sh
roslaunch rtabmap_pkg rtab_rs435.launch
```
### [ORB-SLAM3](https://github.com/thien94/orb_slam3_ros/tree/master)
```bash
bash scripts/install_deps_orbslam3.sh
roslaunch orb_slam3_ros tum_rgbd.launch
```

## Preception

Perception Module.

### [Grounded SAM-2](https://github.com/yutian929/YanBot-Sense_Grounded_SAM_2)
```bash
bash scripts/install_deps_grounded_sam2.sh
# Then follow the instructions in src/perception/grounded_sam2/REAMDE.md [Installation with docker]
```

###


## Data Prepare
### [roboTHOR/AI2THOR](https://ai2thor.allenai.org/robothor/documentation/)
```bash
rosrun robothor_pkg robothor_node.py
```
### [TUM](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download#)
```bash
# Visit [TUM]("https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download#"), and download the rosbag you need.

rosbag play rgbd_dataset_freiburg1_xyz.bag
```
