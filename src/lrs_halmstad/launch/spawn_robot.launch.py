from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path

#from lrs_util import XacroContents
import launch_ros
#import xacro

def generate_launch_description():
    urdf_path = get_package_share_path('lrs_halmstad') / 'urdf'
    sdf_path = get_package_share_path('lrs_halmstad') / 'sdf'
    default_urdf_model_path = urdf_path / 'lrs_piraya.urdf.xacro'
    default_sdf_model_path = sdf_path / 'lrs_piraya.sdf'

    world_arg = DeclareLaunchArgument(name='world', default_value='empty',
                                      description='World to spawn in')
    
    name_arg = DeclareLaunchArgument(name='name', default_value='piraya0',
                                     description='Name of model')
    
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_sdf_model_path),
                                      description='Absolute path to robot file')

    type_arg = DeclareLaunchArgument(name='type', default_value="piraya",
                                     description='Type of model')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    R = LaunchConfiguration('R')
    P = LaunchConfiguration('P')
    Y = LaunchConfiguration('Y')

    spawn_node = Node(
            package="ros_gz_sim",
            executable='create',
            name='spawn_entity',
            arguments=[ 
                '-world', LaunchConfiguration('world'),               
                '-name', LaunchConfiguration('name'),
#                '-robot_namespace', LaunchConfiguration('name'),
#                '-topic', ['/', LaunchConfiguration('name'), '/robot_description']
#                '-file', LaunchConfiguration('model'),
#                '-file', "/tmp/gen.sdf",
                '-string', Command(["ros2 run lrs_halmstad", " ", "generate_sdf", " ", "--ros-args", " -p type:=", LaunchConfiguration('type'), " -p name:=", LaunchConfiguration('name'), " -p robot:=True"]),
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-R', LaunchConfiguration('R'),
                '-P', LaunchConfiguration('P'),
                '-Y', LaunchConfiguration('Y'),
                '--ros-args', '--log-level', 'info'
            ]
        )

    

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value="0.0"),
        DeclareLaunchArgument('y', default_value="0.0"),
        DeclareLaunchArgument('z', default_value="0.0"),
        DeclareLaunchArgument('R', default_value="0.0"),
        DeclareLaunchArgument('P', default_value="0.0"),
        DeclareLaunchArgument('Y', default_value="0.0"),

        type_arg,
        model_arg,
        world_arg,        
        name_arg,
        spawn_node
    ])

