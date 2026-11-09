"""Real AMIR hardware, initial-position controllers and frozen snapshot receiver.

The external AMIR bridge must publish encoder_pub and consume motor_sub.
Switch arm_controller to both forward controllers before starting QP.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def arg(name, default, description):
        return DeclareLaunchArgument(name, default_value=default, description=description)

    target_frame = LaunchConfiguration('kinematics_frame')
    frozen_topic = LaunchConfiguration('frozen_cbf_topic')
    description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', LaunchConfiguration('model'),
        ' ros2_control_hardware_type:=', LaunchConfiguration('hardware_type'),
        ' initial_positions_file:=', PathJoinSubstitution([
            FindPackageShare('amir_moveit_config'), 'config', 'initial_positions.yaml']),
    ], on_stderr='warn'), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=PathJoinSubstitution([
            FindPackageShare('amir_moveit_config'), 'config', 'amir_mecanum3.urdf.xacro'])),
        arg('hardware_type', 'amir_hardware', 'amir_hardware for real robot; mock_components for checks'),
        arg('snapshot_topic', '/cbf/octomap_snapshot', 'One-shot map sent by mapping PC'),
        arg('frozen_cbf_topic', '/cbf/frozen_obstacle_spheres', 'Robot-frame frozen CBF map'),
        arg('kinematics_frame', 'link0_1', 'QP arm base frame'),
        arg('publish_camera_mount_tf', 'true', 'Publish Gazebo-equivalent camera mount TF'),
        arg('camera_parent_frame', 'link1_1', 'Gazebo camera mount parent'),
        arg('camera_frame', 'camera_link', 'RealSense body frame; optical TF comes from camera driver'),
        arg('camera_x', '0.0', 'Mount translation x in meters'),
        arg('camera_y', '0.07', 'Mount translation y in meters'),
        arg('camera_z', '0.05', 'Mount translation z in meters'),
        arg('camera_roll', '0.0', 'Mount roll in radians'),
        arg('camera_pitch', '0.0', 'Mount pitch in radians'),
        arg('camera_yaw', '1.57', 'Mount yaw in radians'),
        Node(
            package='controller_manager', executable='ros2_control_node',
            output='screen', parameters=[{'robot_description': description, 'use_sim_time': False},
                PathJoinSubstitution([FindPackageShare('amir_driver'), 'config', 'amir_ros2_controllers.yaml'])],
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen', parameters=[{'robot_description': description, 'use_sim_time': False}],
        ),
        Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=['joint_state_broadcaster', 'arm_controller', 'gripper_controller',
                       '-c', '/controller_manager', '--controller-manager-timeout', '60'],
        ),
        Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=['forward_position_controller', 'forward_velocity_controller', '--inactive',
                       '-c', '/controller_manager', '--controller-manager-timeout', '60'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='amir_realsense_mount_tf', output='screen',
            condition=IfCondition(LaunchConfiguration('publish_camera_mount_tf')),
            arguments=[
                '--x', LaunchConfiguration('camera_x'),
                '--y', LaunchConfiguration('camera_y'),
                '--z', LaunchConfiguration('camera_z'),
                '--roll', LaunchConfiguration('camera_roll'),
                '--pitch', LaunchConfiguration('camera_pitch'),
                '--yaw', LaunchConfiguration('camera_yaw'),
                '--frame-id', LaunchConfiguration('camera_parent_frame'),
                '--child-frame-id', LaunchConfiguration('camera_frame'),
            ],
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='amir_driver', executable='cbf_octomap_snapshot.py',
            name='amir_cbf_snapshot_receiver', output='screen',
            parameters=[{
                'mode': 'receive', 'input_topic': LaunchConfiguration('snapshot_topic'),
                'output_topic': frozen_topic, 'target_frame': target_frame,
                'use_sim_time': False,
            }],
        ),
    ])
