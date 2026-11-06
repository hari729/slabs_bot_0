from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    pkg_path = get_package_share_path('mearm_model')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(pkg_path / 'urdf' / 'robot.urdf'),
        description='Path to robot URDF/xacro file'
    )

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Use joint_state_publisher_gui if true'
    )

    rvizconfig_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(pkg_path / 'rviz' / 'urdf.rviz'),
        description='Path to RViz config file'
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        gui_arg,
        rvizconfig_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node
    ])
