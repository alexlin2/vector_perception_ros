#!/bin/bash

# Colors for better output
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Starting build process for vector_perception_ros2...${NC}"

# Setup workspace paths
PACKAGE_DIR=~/ros2_ws/src/vector_perception_ros2
INSTALL_DIR=~/ros2_ws/install/vector_perception_ros2

# Fix package.xml
echo -e "${BLUE}Updating package.xml...${NC}"
cat > /tmp/tmp_package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vector_perception_ros2</name>
  <version>0.0.0</version>
  <description>ROS2 interface for Vector Perception segmentation and 3D perception</description>
  <maintainer email="alex.lin416@outlook.com">alexlin</maintainer>
  <license>TODO: License declaration</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>vector_perception</depend>
  <depend>cv_bridge</depend>
  <depend>message_filters</depend>
  <depend>tf2_ros</depend>

  <build_depend>ament_cmake</build_depend>
  <build_depend>ament_cmake_python</build_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

cp /tmp/tmp_package.xml ./package.xml

# Build the package
cd ~/ros2_ws
echo -e "${BLUE}Building package...${NC}"
colcon build --packages-select vector_perception_ros2 --event-handlers console_direct+

# Source the workspace
source install/setup.bash

# Check if lib directory exists, if not create it
if [ ! -d "$INSTALL_DIR/lib/vector_perception_ros2" ]; then
    echo -e "${BLUE}Creating lib directory...${NC}"
    mkdir -p $INSTALL_DIR/lib/vector_perception_ros2
fi

# Manually copy the script
echo -e "${BLUE}Copying executable script...${NC}"
cp $PACKAGE_DIR/scripts/segment_3d_node.py $INSTALL_DIR/lib/vector_perception_ros2/segment_3d_node
chmod +x $INSTALL_DIR/lib/vector_perception_ros2/segment_3d_node

echo -e "${GREEN}Build complete! You can now run:${NC}"
echo -e "${BLUE}ros2 launch vector_perception_ros2 segment_3d.launch.py${NC}"