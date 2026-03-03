from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _estimator_condition():
    start_arg = LaunchConfiguration('start_leader_estimator')
    leader_mode = LaunchConfiguration('leader_mode')
    perception = LaunchConfiguration('leader_perception_enable')
    yolo_weights = LaunchConfiguration('yolo_weights')
    return IfCondition(
        PythonExpression([
            "(",
            "'", start_arg, "'.lower() in ('1','true','yes','on')",
            ") or (",
            "'", start_arg, "'.lower() == 'auto' and (",
            "'", leader_mode, "'.lower() in ('pose','estimate')",
            " or ",
            "'", perception, "'.lower() in ('1','true','yes','on')",
            " or ",
            "'", yolo_weights, "'.strip() != ''",
            "))",
        ])
    )


def _controller_backend_condition():
    backend = LaunchConfiguration('uav_backend')
    return IfCondition(
        PythonExpression([
            "'", backend, "'.lower() == 'controller'",
        ])
    )


def _simulator_failfast_condition():
    return _controller_backend_condition()


def generate_launch_description():
    params_default = PathJoinSubstitution(
        [FindPackageShare('lrs_halmstad'), 'config', 'run_round_follow_defaults.yaml']
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_default,
        description='Parameter YAML for leader_estimator, follow_uav, and UGV motion driver',
    )
    world_arg = DeclareLaunchArgument('world', default_value='orchard')
    uav_name_arg = DeclareLaunchArgument('uav_name', default_value='dji0')
    leader_mode_arg = DeclareLaunchArgument('leader_mode', default_value='odom')
    uav_backend_arg = DeclareLaunchArgument(
        'uav_backend',
        default_value='setpose',
        description="UAV backend mode: setpose (baseline) or controller (simulator command interface)",
    )
    leader_perception_enable_arg = DeclareLaunchArgument('leader_perception_enable', default_value='false')
    start_estimator_arg = DeclareLaunchArgument(
        'start_leader_estimator',
        default_value='auto',
        description="auto|true|false; auto starts estimator for pose/estimate, perception mode, or when yolo_weights is set",
    )
    ugv_cmd_topic_arg = DeclareLaunchArgument('ugv_cmd_topic', default_value='/a201_0000/cmd_vel')
    ugv_odom_topic_arg = DeclareLaunchArgument('ugv_odom_topic', default_value='/a201_0000/platform/odom')
    leader_image_topic_arg = DeclareLaunchArgument('leader_image_topic', default_value=['/', LaunchConfiguration('uav_name'), '/camera0/image_raw'])
    leader_camera_info_topic_arg = DeclareLaunchArgument('leader_camera_info_topic', default_value=['/', LaunchConfiguration('uav_name'), '/camera0/camera_info'])
    leader_depth_topic_arg = DeclareLaunchArgument('leader_depth_topic', default_value='')
    leader_uav_pose_topic_arg = DeclareLaunchArgument('leader_uav_pose_topic', default_value=['/', LaunchConfiguration('uav_name'), '/pose_cmd'])
    backend_frame_id_arg = DeclareLaunchArgument('backend_frame_id', default_value='map')
    backend_initial_x_arg = DeclareLaunchArgument('backend_initial_x', default_value='-2.0')
    backend_initial_y_arg = DeclareLaunchArgument('backend_initial_y', default_value='0.0')
    backend_initial_z_arg = DeclareLaunchArgument('backend_initial_z', default_value='5.0')
    backend_initial_yaw_deg_arg = DeclareLaunchArgument('backend_initial_yaw_deg', default_value='0.0')
    backend_initial_pan_deg_arg = DeclareLaunchArgument('backend_initial_pan_deg', default_value='0.0')
    backend_initial_tilt_deg_arg = DeclareLaunchArgument('backend_initial_tilt_deg', default_value='-45.0')
    yolo_weights_arg = DeclareLaunchArgument('yolo_weights', default_value='')
    yolo_device_arg = DeclareLaunchArgument('yolo_device', default_value='cpu')
    event_topic_arg = DeclareLaunchArgument('event_topic', default_value='/coord/events')
    # Fallback delay only. The primary startup gating now happens inside
    # ugv_motion_driver via a lightweight readiness check (cmd subscriber + odom flow).
    ugv_start_delay_arg = DeclareLaunchArgument('ugv_start_delay_s', default_value='0.0')
    shutdown_when_ugv_done_arg = DeclareLaunchArgument(
        'shutdown_when_ugv_done',
        default_value='false',
        description='If true, stop this launch when ugv_motion_driver exits.',
    )

    estimator_node = Node(
        package='lrs_halmstad',
        executable='leader_estimator',
        name='leader_estimator',
        output='screen',
        condition=_estimator_condition(),
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'uav_name': LaunchConfiguration('uav_name'),
                'camera_topic': LaunchConfiguration('leader_image_topic'),
                'camera_info_topic': LaunchConfiguration('leader_camera_info_topic'),
                'depth_topic': LaunchConfiguration('leader_depth_topic'),
                'uav_pose_topic': LaunchConfiguration('leader_uav_pose_topic'),
                'device': LaunchConfiguration('yolo_device'),
                'yolo_weights': LaunchConfiguration('yolo_weights'),
                'event_topic': LaunchConfiguration('event_topic'),
            },
        ],
    )

    follow_node = Node(
        package='lrs_halmstad',
        executable='follow_uav',
        name='follow_uav',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'world': LaunchConfiguration('world'),
                'uav_name': LaunchConfiguration('uav_name'),
                'leader_input_type': LaunchConfiguration('leader_mode'),
                'leader_odom_topic': LaunchConfiguration('ugv_odom_topic'),
                'event_topic': LaunchConfiguration('event_topic'),
                'uav_backend': LaunchConfiguration('uav_backend'),
                'controller_seed_x': LaunchConfiguration('backend_initial_x'),
                'controller_seed_y': LaunchConfiguration('backend_initial_y'),
                'controller_seed_yaw_deg': LaunchConfiguration('backend_initial_yaw_deg'),
            },
        ],
    )

    simulator_backend_node = Node(
        package='lrs_halmstad',
        executable='simulator',
        name='simulator',
        namespace=LaunchConfiguration('uav_name'),
        output='screen',
        condition=_controller_backend_condition(),
        parameters=[
            {
                'world': LaunchConfiguration('world'),
                'name': LaunchConfiguration('uav_name'),
                'frame_id': LaunchConfiguration('backend_frame_id'),
                'initial_x': LaunchConfiguration('backend_initial_x'),
                'initial_y': LaunchConfiguration('backend_initial_y'),
                'initial_z': LaunchConfiguration('backend_initial_z'),
                'initial_yaw_deg': LaunchConfiguration('backend_initial_yaw_deg'),
                'initial_pan_deg': LaunchConfiguration('backend_initial_pan_deg'),
                'initial_tilt_deg': LaunchConfiguration('backend_initial_tilt_deg'),
            },
        ],
    )

    ugv_node = Node(
        package='lrs_halmstad',
        executable='ugv_motion_driver',
        name='ugv_motion_driver',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'cmd_topic': LaunchConfiguration('ugv_cmd_topic'),
                'start_delay_s': LaunchConfiguration('ugv_start_delay_s'),
                'ready_check_enable': True,
                'ready_require_cmd_subscriber': True,
                'ready_require_odom_flow': True,
                'ready_odom_topic': LaunchConfiguration('ugv_odom_topic'),
            },
        ],
    )

    ugv_delayed_start = TimerAction(
        period=0.1,
        actions=[
            LogInfo(msg='[run_round_follow_motion] Starting UGV motion driver (follow stack already launched)'),
            ugv_node,
        ],
    )

    control_chain_log = LogInfo(
        msg=[
            '[run_round_follow_motion] Control chain: leader_mode=',
            LaunchConfiguration('leader_mode'),
            ' -> follow_uav intent=/',
            LaunchConfiguration('uav_name'),
            '/pose_cmd -> backend=',
            LaunchConfiguration('uav_backend'),
            ' (no manual UAV command path in this launch)',
        ],
    )

    shutdown_on_ugv_done = RegisterEventHandler(
        OnProcessExit(
            target_action=ugv_node,
            on_exit=[
                LogInfo(msg='[run_round_follow_motion] ugv_motion_driver exited; shutting down launch'),
                EmitEvent(event=Shutdown(reason='UGV motion complete')),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('shutdown_when_ugv_done')),
    )
    shutdown_on_simulator_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=simulator_backend_node,
            on_exit=[
                LogInfo(msg='[run_round_follow_motion] simulator backend exited in controller mode; shutting down launch'),
                EmitEvent(event=Shutdown(reason='Simulator backend exited')),
            ],
        ),
        condition=_simulator_failfast_condition(),
    )

    return LaunchDescription([
        params_file_arg,
        world_arg,
        uav_name_arg,
        leader_mode_arg,
        uav_backend_arg,
        leader_perception_enable_arg,
        start_estimator_arg,
        ugv_cmd_topic_arg,
        ugv_odom_topic_arg,
        leader_image_topic_arg,
        leader_camera_info_topic_arg,
        leader_depth_topic_arg,
        leader_uav_pose_topic_arg,
        backend_frame_id_arg,
        backend_initial_x_arg,
        backend_initial_y_arg,
        backend_initial_z_arg,
        backend_initial_yaw_deg_arg,
        backend_initial_pan_deg_arg,
        backend_initial_tilt_deg_arg,
        yolo_weights_arg,
        yolo_device_arg,
        event_topic_arg,
        ugv_start_delay_arg,
        shutdown_when_ugv_done_arg,
        control_chain_log,
        estimator_node,
        follow_node,
        simulator_backend_node,
        ugv_delayed_start,
        shutdown_on_ugv_done,
        shutdown_on_simulator_exit,
    ])
