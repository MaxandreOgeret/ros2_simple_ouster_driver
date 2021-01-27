from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    composable_nodes = [
        ComposableNode(
            package='ros2_simple_ouster_driver',
            plugin='ros2_simple_ouster_driver::Driver',
        )
    ]

    composable_node_container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='simple_ouster_driver',
        namespace='',
        composable_node_descriptions=composable_nodes
    )

    return LaunchDescription([
        composable_node_container
    ])

