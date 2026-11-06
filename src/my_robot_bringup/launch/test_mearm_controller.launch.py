from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    robot_description_path = get_package_share_path('my_robot_description')
    robot_bringup_path = get_package_share_path('my_robot_bringup')

    urdf_path = os.path.join(robot_description_path, 'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(robot_description_path, 'rviz', 'urdf_config.rviz')
    robot_controllers = os.path.join(robot_bringup_path, 'config', 'my_robot_controllers.yaml')

    mount_x_arg = DeclareLaunchArgument(
        'mearm_mount_x', default_value='0.09',
        description='X offset of the MeArm base relative to top_cover',
    )
    mount_y_arg = DeclareLaunchArgument(
        'mearm_mount_y', default_value='0.0',
        description='Y offset of the MeArm base relative to top_cover',
    )
    mount_z_arg = DeclareLaunchArgument(
        'mearm_mount_z', default_value='0.01',
        description='Z offset of the MeArm base relative to top_cover',
    )
    mount_yaw_arg = DeclareLaunchArgument(
        'mearm_mount_yaw', default_value='0.0',
        description='Yaw rotation of the MeArm base relative to top_cover',
    )

    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' use_esp32_hardware:=false',
            ' use_mearm_mock_hardware:=true',
            ' mearm_mount_x:=', LaunchConfiguration('mearm_mount_x'),
            ' mearm_mount_y:=', LaunchConfiguration('mearm_mount_y'),
            ' mearm_mount_z:=', LaunchConfiguration('mearm_mount_z'),
            ' mearm_mount_yaw:=', LaunchConfiguration('mearm_mount_yaw'),
        ]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    joint_group_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_position_controller'],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        mount_x_arg,
        mount_y_arg,
        mount_z_arg,
        mount_yaw_arg,
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        joint_group_position_controller_spawner,
        rviz_node,
    ])
