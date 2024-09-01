from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        # Declare launch arguments for the namespace
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the nodes'
        ),
        
        # Launch the nodes in the specified order
        Node(
            package='ERPB_robot_agent',
            executable='environmental_information_publisher_node',
            output='screen'
        ),
        Node(
            package='ERPB_robot_agent',
            executable='task_broadcaster_node',
            output='screen'
        ),
       ])
