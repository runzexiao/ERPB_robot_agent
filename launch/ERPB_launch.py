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
            executable='start_node',
            namespace=namespace,
            output='screen',
            arguments=[namespace]  # 这里传递 namespace 作为参数
        ),
        Node(
            package='ERPB_robot_agent',
            executable='operator_task_processing_node',
            namespace=namespace,
            output='screen',
            arguments=[namespace]
        ),
        Node(
            package='ERPB_robot_agent',
            executable='bidding_node',
            namespace=namespace,
            output='screen',
            arguments=[namespace]
        ),
        Node(
            package='ERPB_robot_agent',
            executable='task_decomposition_node',
            namespace=namespace,
            output='screen',
            arguments=[namespace]
        ),
        Node(
            package='ERPB_robot_agent',
            executable='task_manager_node',
            namespace=namespace,
            output='screen',
            arguments=[namespace]
        ),
        Node(
            package='ERPB_robot_agent',
            executable='bidding_evaluation_node',
            namespace=namespace,
            output='screen',
            arguments=[namespace]
        ),
        Node(
            package='ERPB_robot_agent',
            executable='execute_node',
            namespace=namespace,
            output='screen',
            arguments=[namespace]
        ),
    ])
