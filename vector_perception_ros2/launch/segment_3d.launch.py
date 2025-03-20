from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import GroupAction

def generate_launch_description():
    # Declare launch arguments
    return LaunchDescription([
        
        # Topic configurations
        DeclareLaunchArgument(
            'color_image_topic',
            default_value='/camera/color/image_raw',
            description='Color image topic'
        ),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/camera/depth/image_raw',
            description='Depth image topic'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
            description='Camera info topic'
        ),
        DeclareLaunchArgument(
            'depth_camera_info_topic',
            default_value='/camera/depth/camera_info',
            description='Depth info topic'
        ),
        
        # Frame configurations
        DeclareLaunchArgument(
            'output_frame',
            default_value='camera_color_optical_frame',
            description='Output frame for detections'
        ),
        
        # Detection configurations
        DeclareLaunchArgument(
            'min_points',
            default_value='10',
            description='Minimum number of points required for 3D detection'
        ),
        
        # Feature toggles
        DeclareLaunchArgument(
            'enable_tracking',
            default_value='true',
            description='Enable object tracking'
        ),
        DeclareLaunchArgument(
            'enable_analysis',
            default_value='false',
            description='Enable image analysis'
        ),
        DeclareLaunchArgument(
            'visualization_enabled',
            default_value='true',
            description='Enable detection visualization'
        ),
        
        # Analysis configurations
        DeclareLaunchArgument(
            'min_analysis_interval',
            default_value='4.0',
            description='Minimum time between analyses in seconds'
        ),

        # Create a group with namespace
        GroupAction(
            actions=[
                # Sam3D node
                Node(
                    package='vector_perception_ros2',  # Replace with your package name
                    executable='segment_3d_node',
                    name='segment_3d_node',
                    output='screen',
                    emulate_tty=True,
                    parameters=[{
                        'color_image_topic': [
                            LaunchConfiguration('color_image_topic')
                        ],
                        'depth_image_topic': [
                            LaunchConfiguration('depth_image_topic')
                        ],
                        'camera_info_topic': [
                            LaunchConfiguration('camera_info_topic')
                        ],
                        'depth_camera_info_topic': [
                            LaunchConfiguration('depth_camera_info_topic')
                        ],
                        'output_frame': LaunchConfiguration('output_frame'),
                        'min_points': LaunchConfiguration('min_points'),
                        'enable_tracking': LaunchConfiguration('enable_tracking'),
                        'enable_analysis': LaunchConfiguration('enable_analysis'),
                        'visualization_enabled': LaunchConfiguration('visualization_enabled'),
                        'min_analysis_interval': LaunchConfiguration('min_analysis_interval'),
                    }],
                    # Additional node configurations if needed
                    # remappings=[],
                    # arguments=[],
                )
            ]
        )
    ])