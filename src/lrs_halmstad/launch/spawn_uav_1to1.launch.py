import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _gazebo_world_name(world_sub):
    return PythonExpression([
        "'office_construction' if '",
        world_sub,
        "' == 'construction' else '",
        world_sub,
        "'",
    ])


def generate_launch_description():
    world_arg = DeclareLaunchArgument('world', default_value='warehouse')
    uav_name_arg = DeclareLaunchArgument('uav_name', default_value='dji0')
    uav_mode_arg = DeclareLaunchArgument('uav_mode', default_value='teleport')
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='-2.0',
    )
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
    )
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='7.0',
    )
    camera_name_arg = DeclareLaunchArgument('camera_name', default_value='camera0')
    uav_camera_mode_arg = DeclareLaunchArgument('uav_camera_mode', default_value='integrated_joint')
    camera_pitch_offset_deg_arg = DeclareLaunchArgument('camera_pitch_offset_deg', default_value='45.0')
    share_dir = get_package_share_directory('lrs_halmstad')
    gz_world = _gazebo_world_name(LaunchConfiguration('world'))

    uav_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_dir, 'spawn_robot.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'name': LaunchConfiguration('uav_name'),
            'type': 'm100',
            'uav_mode': LaunchConfiguration('uav_mode'),
            'with_camera': 'true',
            'bridge_camera': 'true',
            'bridge_gimbal': 'true',
            'camera_pitch_offset_deg': LaunchConfiguration('camera_pitch_offset_deg'),
            'camera_name': LaunchConfiguration('camera_name'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
        }.items(),
    )

    set_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            ['/world/', gz_world, '/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        uav_name_arg,
        uav_mode_arg,
        x_arg,
        y_arg,
        z_arg,
        camera_name_arg,
        uav_camera_mode_arg,
        camera_pitch_offset_deg_arg,
        set_pose_bridge,
        uav_spawn,
    ])
