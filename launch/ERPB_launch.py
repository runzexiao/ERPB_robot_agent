from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取功能包的共享目录路径
    my_package_share_dir = get_package_share_directory('ERPB_robot_agent')

    # 定义参数文件路径
    param_file = os.path.join(my_package_share_dir, 'config', 'kcafe1_params.yaml')

    print(f'Parameter file path: {param_file}')
    return LaunchDescription([
        # Declare launch arguments for the namespace
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the node'
        ),
        
        # Load parameters and start the node with namespace
        Node(
            package='ERPB_robot_agent',
            executable='test_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[param_file],
            output='screen'
        )
    ])

