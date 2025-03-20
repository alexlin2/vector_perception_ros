cmake_minimum_required(VERSION 3.8)
project(vector_perception_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Object3D.msg"
  "msg/Object3DArray.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

ament_package()
