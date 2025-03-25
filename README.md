# Vector Perception ROS2

ROS2 interface for Vector Perception AI segmentation and 3D perception with AnyGrasp integration for robotic grasping.

## Features

- 2D image segmentation using SAM (Segment Anything Model)
- 3D point cloud generation from segmented objects
- Object tracking and analysis
- Grasp detection using AnyGrasp SDK
- Custom ROS2 messages for 3D object representation

## Custom Messages

This package provides two custom message types:

1. `Object3D`: Represents a single 3D object with:
   - Point cloud
   - RGB color
   - 2D mask
   - Bounding box
   - Target ID (for tracking)
   - Object name
   - Detection probability

2. `Object3DArray`: Contains an array of `Object3D` messages

## Prerequisites

### AnyGrasp License
Please refer to `anygrasp_external_usage.md` to obtain the AnyGrasp license before using this package. The license files should be placed in the `vector_perception_ros2/vector_perception_ros2/license/` directory.

### Vector Perception
Install Vector Perception from the GitHub repository:
```bash
pip install git+https://github.com/alexlin2/vector_perception.git
```

### Dependencies
- ROS2 (Humble or later recommended)
- PyTorch with CUDA
- Open3D
- OpenCV
- NumPy

## Building

This is a Python-only package that also generates custom messages.

```bash
# Clone the repository
cd ~/ros2_ws/src
git clone https://github.com/alexlin2/vector_perception_ros.git

# Build the package
cd ~/ros2_ws
colcon build --packages-select vector_perception_ros2 vector_perception_msgs

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

## Usage

### Launch Files

The package provides a launch file for easy setup:

```bash
# Launch the segmentation node with default parameters
ros2 launch vector_perception_ros2 segment_3d.launch.py
```

### Available Parameters

You can customize the node with these parameters:

```bash
# Manual node launch with custom parameters
ros2 run vector_perception_ros2 segment_3d_node \
  --ros-args \
  -p color_image_topic:=/camera/color/image_raw \
  -p depth_image_topic:=/camera/depth/image_raw \
  -p camera_info_topic:=/camera/color/camera_info \
  -p depth_camera_info_topic:=/camera/depth/camera_info \
  -p output_frame:=camera_color_optical_frame \
  -p min_points:=10 \
  -p enable_tracking:=true \
  -p enable_analysis:=true \
  -p visualization_enabled:=true \
  -p min_analysis_interval:=4.0
```

## Topics

The node publishes on the following topics:

- `/segmented_objects` (sensor_msgs/PointCloud2): Combined point cloud of all segmented objects
- `/detection_visualization` (sensor_msgs/Image): Image with visualization of detected objects
- `/detected_objects` (vector_perception_ros2/Object3DArray): Array of detected objects with detailed information

## Grasp Detection

For grasp detection functionality using AnyGrasp:

```bash
# Launch the grasp detection node 
ros2 run vector_perception_ros2 grasp_node
```

More information about the AnyGrasp integration can be found in the `anygrasp_external_usage.md` file.
