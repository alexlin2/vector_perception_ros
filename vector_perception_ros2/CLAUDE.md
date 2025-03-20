# ROS2 Development Guidelines for vector_perception_ros2

## Build Commands
- Build: `colcon build --packages-select vector_perception_ros2`
- Clean and rebuild: `colcon clean --packages-select vector_perception_ros2`
- Launch segmentation: `ros2 launch vector_perception_ros2 segment_3d.launch.py`

## Test Commands
- Run all tests: `colcon test --packages-select vector_perception_ros2`
- Run single test: `python3 -m pytest src/vector_perception_ros2/test/test_flake8.py -v`
- Lint: `ament_flake8 src/vector_perception_ros2`
- Check PEP257: `ament_pep257 src/vector_perception_ros2`

## Code Style Guidelines
- Follow PEP 8 style guide and PEP 257 for docstrings
- Imports order: standard libraries → ROS2 packages → project modules
- Naming: snake_case for functions/variables, PascalCase for classes
- Use type hints for function parameters and return values
- 4-space indentation, 100 char line length
- Error handling: use try/except blocks with specific exceptions
- Include copyright headers in all source files
- Document node interfaces (publishers, subscribers, services, parameters)