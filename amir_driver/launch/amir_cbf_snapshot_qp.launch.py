"""CBF-QP and interactive RViz; start amir_bringup.launch.py first."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def arg(name, default, description):
        return DeclareLaunchArgument(name, default_value=default, description=description)

    def number(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    target_frame = LaunchConfiguration('kinematics_frame')
    frozen_topic = LaunchConfiguration('frozen_cbf_topic')
    return LaunchDescription([
        arg('frozen_cbf_topic', '/cbf/frozen_obstacle_spheres', 'Frozen CBF map from amir_bringup'),
        arg('kinematics_frame', 'link0_1', 'Must match amir_bringup kinematics_frame'),
        arg('joint_state_topic', '/joint_states', 'Measured real joint states'),
        arg('target_topic', '/ee_target_pose', 'QP target PoseStamped'),
        arg('command_topic', '/forward_position_controller/commands', 'Five-joint position controller input'),
        arg('velocity_command_topic', '/forward_velocity_controller/commands', 'Five-joint velocity input in rad/s'),
        arg('joint_speed_mode', 'qp', 'qp: original solver velocities; fixed: configured speed magnitudes'),
        arg('position_command_mode', 'integrated', 'integrated: bounded command integration; measured: original measured-position steps'),
        arg('max_position_lead', '0.02', 'Maximum position command lead from measured joints in rad'),
        DeclareLaunchArgument('joint_speed_config', default_value=PathJoinSubstitution([
            FindPackageShare('amir_qp_control'), 'config', 'joint_speeds.yaml'])),
        arg('qp_enabled', 'true', 'Enable QP solving; false continuously stops real hardware'),
        arg('tracking_weight_x', '1.0', 'Cartesian x tracking cost weight in kinematics_frame'),
        arg('tracking_weight_y', '1.0', 'Cartesian y tracking cost weight in kinematics_frame'),
        arg('tracking_weight_z', '1.0', 'Cartesian z tracking cost weight in kinematics_frame'),
        arg('max_cartesian_velocity', '0.1', 'QP Cartesian velocity limit in m/s'),
        arg('max_joint_velocity', '4.0', 'QP joint velocity limit in rad/s'),
        arg('use_rviz', 'true', 'Show frozen CBF map and robot'),
        arg('use_rviz_target', 'true', 'Start interactive RViz target with continuous drag tracking'),
        arg('target_initial_x', '0', 'Initial target marker x in kinematics_frame (m)'),
        arg('target_initial_y', '0.3', 'Initial target marker y in kinematics_frame (m)'),
        arg('target_initial_z', '0.2', 'Initial target marker z in kinematics_frame (m)'),
        arg('target_drag_update_rate', '30.0', 'Maximum target updates per second while dragging'),
        Node(
            package='amir_driver', executable='rviz_ee_target.py',
            name='amir_cbf_rviz_ee_target', output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz_target')),
            parameters=[{
                'use_sim_time': False,
                'frame_id': target_frame,
                'target_topic': LaunchConfiguration('target_topic'),
                'clicked_point_topic': '/clicked_point',
                'interactive_marker_namespace': 'ee_target_marker',
                'initial_x': number('target_initial_x'),
                'initial_y': number('target_initial_y'),
                'initial_z': number('target_initial_z'),
                'marker_scale': 0.16,
                'publish_while_dragging': True,
                'drag_update_rate': number('target_drag_update_rate'),
            }],
        ),
        Node(
            package='amir_qp_control', executable='qp_tracking_controller.py',
            name='qp_tracking_controller', output='screen',
            parameters=[LaunchConfiguration('joint_speed_config'), {
                'use_sim_time': False,
                'joint_names': ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4'],
                'qp_enabled': ParameterValue(LaunchConfiguration('qp_enabled'), value_type=bool),
                'joint_state_topic': LaunchConfiguration('joint_state_topic'),
                'target_topic': LaunchConfiguration('target_topic'),
                'command_topic': LaunchConfiguration('command_topic'),
                'velocity_command_topic': LaunchConfiguration('velocity_command_topic'),
                'joint_speed_mode': LaunchConfiguration('joint_speed_mode'),
                'position_command_mode': LaunchConfiguration('position_command_mode'),
                'max_position_lead': number('max_position_lead'),
                'kinematics_frame': target_frame,
                'control_frequency': 50.0, 'kp': 4.0, 'regularization': 1e-3,
                'tracking_weight_x': number('tracking_weight_x'),
                'tracking_weight_y': number('tracking_weight_y'),
                'tracking_weight_z': number('tracking_weight_z'),
                'max_cartesian_velocity': number('max_cartesian_velocity'),
                'max_joint_velocity': number('max_joint_velocity'),
                'position_tolerance': 0.005,
                'stop_after_reaching_target': False,
                'cbf_enabled': True, 'cbf_require_obstacle_map': True,
                'cbf_obstacle_topic': frozen_topic,
                'enable_cbf_after_reaching_target': False,
                'arm_cbf_radii': [0.057] * 8,
                'cbf_safety_margin': 0.03, 'cbf_activation_distance': 0.08,
                'cbf_alpha': 5.0, 'max_active_cbf_constraints': 128,
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', name='amir_cbf_snapshot_rviz',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('amir_driver'), 'rviz', 'amir_cbf_snapshot.rviz',
            ]), '-f', target_frame],
            remappings=[('/cbf/frozen_obstacle_spheres', frozen_topic)],
            parameters=[{'use_sim_time': False}],
        ),
    ])
