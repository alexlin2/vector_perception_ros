# Vector Perception ROS2

ROS2 interface for Vector Perception AI segmentation and 3D perception.

## Features

- 2D image segmentation using SAM (Segment Anything Model)
- 3D point cloud generation from segmented objects
- Object tracking and analysis
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

## Building

This is a Python-only package that also generates custom messages.

```bash
# Clone the repository
cd ~/ros2_ws/src
git clone <repository_url>

# Build the package
cd ~/ros2_ws
colcon build --packages-select vector_perception_ros2

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

### Building with Custom Messages

If you're having issues with the package.xml name tag, edit the package.xml file and replace the name tag:

```bash
# Fix the package.xml name tag if needed
nano ~/ros2_ws/src/vector_perception_ros2/package.xml
# Change n tag to name tag for vector_perception_ros2
```

## Usage

```bash
# Launch the segmentation node
ros2 launch vector_perception_ros2 segment_3d.launch.py

# Manual node launch with custom parameters
ros2 run vector_perception_ros2 segment_3d_node \
  --ros-args -p color_image_topic:=/camera/color/image_raw \
  -p depth_image_topic:=/camera/depth/image_raw \
  -p enable_tracking:=true
```

## Topics

The node publishes on the following topics:

- `/segmented_objects` (sensor_msgs/PointCloud2): Combined point cloud of all segmented objects
- `/detection_visualization` (sensor_msgs/Image): Image with visualization of detected objects
- `/detected_objects` (vector_perception_ros2/Object3DArray): Array of detected objects with detailed information (only available if message generation succeeds)