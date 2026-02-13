#import logging
#logging.root.setLevel(logging.DEBUG)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration


world_arg = DeclareLaunchArgument('world', default_value='warehouse',
                      choices=[
                          'construction',
                          'office',
                          'orchard',
                          'pipeline',
                          'solar_farm',
                          'warehouse',
                      ],
                      description='Gazebo World')

def generate_launch_description():
    
    dji0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji0",
                          "type": "m100",
                          "z": "2.27",
                          "world": LaunchConfiguration('world')
                          }.items(),
    )
    
    dji0gimbal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_gimbal.launch.py')),
        launch_arguments={
            "name": "dji0",
            "camera_name": "camera0",
            "type": "m100",
            "z": "2.0",
            "world": LaunchConfiguration('world'),
    }.items(),
    )


    dji1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji1",
                          "type": "m100",
                          "z": "3.27",
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )
    
    dji1gimbal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_gimbal.launch.py')),
        launch_arguments={
            "name": "dji1",
            "camera_name": "camera0",
            "type": "m100",
            "z": "3.0",
            "world": LaunchConfiguration('world'),
    }.items(),
    )

    dji2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji2",
                          "type": "m100",
                          "z": "4.27",
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )
    

    dji2gimbal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_gimbal.launch.py')),
        launch_arguments={
            "name": "dji2",
            "camera_name": "camera0",
            "type": "m100",
            "z": "4.0",
            "world": LaunchConfiguration('world'),
    }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
#        arguments= ['/world/orchard/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        arguments=[
            ['/world/', LaunchConfiguration('world'),'/set_pose@ros_gz_interfaces/srv/SetEntityPose']
        ],
        output='screen'
    )
    
    
    return LaunchDescription([
        world_arg,
        bridge,
        dji0,
        dji0gimbal,
        dji1,
        dji1gimbal,        
        dji2,
        dji2gimbal,        
    ])
