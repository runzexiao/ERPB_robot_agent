import rclpy
from rclpy.node import Node
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch import LaunchService
from multiprocessing import Process

def launch_node(package_name, executable_name, namespace):
    ld = LaunchDescription([
        LaunchNode(
            package=package_name,
            executable=executable_name,
            namespace=namespace,
            output='screen'
        )
    ])
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

class NodeStarter(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('Node Starter has been started.')
        # 启动其他节点
        self.start_other_node('ERPB_robot_agent', 'start_node', 'robot1')
        self.start_other_node('ERPB_robot_agent', 'bidding_node', 'robot1')
        # 创建计时器，每秒调用一次 timer_callback
        self.timer = self.create_timer(1.0, self.timer_callback)

    def start_other_node(self, package_name, executable_name, namespace):
        p = Process(target=launch_node, args=(package_name, executable_name, namespace))
        p.start()

    def timer_callback(self):
        self.get_logger().info('Timer callback: NodeStarter is running.')

def main(args=None):
    rclpy.init(args=args)
    node_starter = NodeStarter()
    rclpy.spin(node_starter)
    node_starter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
