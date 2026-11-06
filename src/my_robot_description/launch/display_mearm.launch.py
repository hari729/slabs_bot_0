from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    robot_description_path = get_package_share_path('my_robot_description')
    urdf_path = os.path.join(robot_description_path, 'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(robot_description_path, 'rviz', 'urdf_config.rviz')

    mount_x_arg = DeclareLaunchArgument(
        'mearm_mount_x',
        default_value='0.08',
        description='X offset of the MeArm base relative to base_link (forward+)',
    )
    mount_y_arg = DeclareLaunchArgument(
        'mearm_mount_y',
        default_value='0.0',
        description='Y offset of the MeArm base relative to base_link (left+)',
    )
    mount_z_arg = DeclareLaunchArgument(
        'mearm_mount_z',
        default_value='0.05',
        description='Z offset of the MeArm base relative to base_link (up+)',
    )
    mount_yaw_arg = DeclareLaunchArgument(
        'mearm_mount_yaw',
        default_value='0.0',
        description='Yaw rotation of the MeArm base relative to base_link (rad)',
    )

    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path,
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

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
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
        joint_state_publisher_gui_node,
        rviz_node,
    ])
