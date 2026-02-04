from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # -----------------------
    # Arguments
    # -----------------------
    sensor_prefix_arg = DeclareLaunchArgument(
        'sensor_prefix',
        default_value='front_lower_window',
        description='Sensor namespace / prefix used for image and depth topics'
    )
    sensor_prefix = LaunchConfiguration('sensor_prefix')

    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='',
        description='Port to start rosbridge websocket server on (leave empty to disable)'
    )
    rosbridge_port = LaunchConfiguration('rosbridge_port')

    # -----------------------
    # YOLO bringup
    # -----------------------
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yolo_bringup'),
                'launch',
                'yolo.launch.py'
            )
        ),
        launch_arguments={
            'input_image_topic': [
                TextSubstitution(text='/'),
                sensor_prefix,
                TextSubstitution(text='/color_resized/image_raw')
            ],
            'model': 'yolov8m-seg.pt',
            'use_3d': 'True',
            'target_frame': [
                sensor_prefix,
                TextSubstitution(text='_depth_frame')
            ],
            'input_depth_topic': [
                TextSubstitution(text='/'),
                sensor_prefix,
                TextSubstitution(text='/depth_resized/image_raw')
            ],
            'input_depth_info_topic': [
                TextSubstitution(text='/'),
                sensor_prefix,
                TextSubstitution(text='/depth_resized/camera_info')
            ],
        }.items()
    )

    # -----------------------
    # rosbridge websocket (conditional)
    # -----------------------
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        ),
        launch_arguments={
            'port': rosbridge_port
        }.items(),
        condition=IfCondition(PythonExpression(["'", rosbridge_port, "' != ''"]))
    )

    # -----------------------
    # LaunchDescription
    # -----------------------
    return LaunchDescription([
        sensor_prefix_arg,
        rosbridge_port_arg,
        rosbridge_launch,
        yolo_launch,
    ])
