# Object3D message for Vector Perception

# Header with timestamp and frame info
std_msgs/Header header

# Object identification
int32 target_id
float32 probability
string name

# 2D information
sensor_msgs/Image mask       # Object mask
int32[4] bbox                # Bounding box [x, y, width, height]

# 3D information
sensor_msgs/PointCloud2 point_cloud   # Object's 3D points
std_msgs/ColorRGBA color              # Color used for visualization