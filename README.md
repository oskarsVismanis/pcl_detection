# pcl_detection

## Usage

### Process point cloud and publish to PlanningScene

```bash
ros2 run pcl_detection perceive_surfaces
```

### Save pointcloud (offline)

- launch realsense node

```bash
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
```

- save pointcloud

```bash
ros2 run pcl_detection save_data
```

### View pointcloud

- with pcl_viewer

```bash
pcl_viever path/to/filename.pcd
```

### Detect planes for mobile manipulator

> [NOTE] Assuming [pmb2_ar4_ros](https://gitlab.edi.lv/edi_robotics/robot_control/pmb2_ar4_ros) is installed

- start mob-man 

```bash
ros2 launch pmb2_ar4_gazebo pmb2_ar4_gazebo.launch.py is_public_sim:=True navigation:=True moveit:=True
```

- start plane detection node

```bash
ros2 run pcl_detection perceive_surfaces
```

## Notes

- to work with realsense, an initial passthrough filtering needs to be done, to cut off all the far-away points

## Installation

- Point Cloud Library (PCL)

```bash
sudo apt install libpcl-dev
```

- (optional) for pcl_viewer install pcl_tools

```bash
sudo apt-get install pcl-tools
```

- ROS 2 dependencies for PCL

```bash
sudo apt-get install ros-humble-pcl-ros
sudo apt-get install ros-humble-pcl-conversions
sudo apt-get install ros-humble-pcl-msgs
```

- ROS 2 dependencies for Intel RealSense

```bash
# install Intel RealSense SDK
sudo apt install ros-humble-librealsense2*

# install Intel RealSense ROS 2 wrapper
sudo apt install ros-humble-realsense2-*
```
