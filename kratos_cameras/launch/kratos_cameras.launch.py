from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        FindPackageShare('kratos_cameras'),
        'config',
        'camera_profiles.yaml'
    ])

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to camera profile parameters file'
    )

    destination_host_arg = DeclareLaunchArgument(
        'destination_host',
        default_value='192.168.1.69',
        description='IP of the machine running the GUI (127.0.0.1 if local)'
    )

    camera_node = Node(
        package='kratos_cameras',
        executable='basic_node',
        name='camera_stream_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'stream.destination_host': LaunchConfiguration('destination_host')},
        ]
    )

    return LaunchDescription([
        params_file_arg,
        destination_host_arg,
        camera_node,
    ])
