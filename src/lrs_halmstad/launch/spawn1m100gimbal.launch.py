import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def get_m100_with_gimbal(name, z):
    res = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_wara_gazebo'),
                'spawn_model.launch.py')),
        launch_arguments={"name": name,
                          "type": "m100",
                          "with_camera": "true",
                          "z": f'{z}'
                          }.items(),
    )
    return res;

def get_m100(name, z):
    res = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_wara_gazebo'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": name,
                          "type": "m100",
                          "z": f'{z}'
                          }.items(),
    )
    return res;

def get_gimbal(name, z):
    res = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_wara_gazebo'),
                'spawn_gimbal.launch.py')),
        launch_arguments={"name": name,
                          "type": "m100",
                          "z": f'{z}'
                          }.items(),
    )
    return res;

def generate_launch_description():
    
    dji0 = get_m100("dji0", 9.0)

    gimb0 = get_gimbal("dji0", 9.0)

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/granso/set_pose@ros_gz_interfaces/srv/SetEntityPose'            
        ],
        output='screen'
    )
    
    
    return LaunchDescription([
        bridge,
        dji0,
        gimb0
    ])
