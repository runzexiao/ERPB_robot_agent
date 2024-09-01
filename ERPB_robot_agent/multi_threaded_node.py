import sys
import os

# 添加当前目录到搜索路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import yaml

class MultiThreadedSubscriberNode(Node):
    def __init__(self):
        super().__init__('multi_threaded_node')
        
        # 创建一个ReentrantCallbackGroup，允许并发回调
        self.callback_group = ReentrantCallbackGroup()

        # 定义第一个订阅者
        self.subscription_1 = self.create_subscription(
            String,
            'topic_1',
            self.topic_1_callback,
            10,
            callback_group=self.callback_group
        )

        # 定义第二个订阅者
        self.subscription_2 = self.create_subscription(
            String,
            'topic_2',
            self.topic_2_callback,
            10,
            callback_group=self.callback_group
        )


        self.my_package_share_dir = get_package_share_directory("ERPB_robot_agent")
        self.yaml_file_path = os.path.join(self.my_package_share_dir, 'config', "dump1_params.yaml")
        
        self.load_params()

        try:
            he = eval("self.add(1,2)")
            print(he)
            self.get_logger().info('Successfully executed the function loaded from YAML.')
        except Exception as e:
            self.get_logger().error(f'Failed to execute the function loaded from YAML: {e}')
        

        self.get_logger().info('Multi-threaded subscriber node has been started.')

    def load_params(self):
        with open(self.yaml_file_path, 'r') as file:
            self.config = yaml.safe_load(file)
            self.method_code = self.config['Transport_items_such_as_soil_and_water_pump']['test_function']
            self.my_function_name = self.config['Transport_items_such_as_soil_and_water_pump']['my_function_name']
        exec(self.method_code)  # 动态加载方法到类中

        # 将定义的函数绑定到 MyRobot 类
        for name in eval(self.my_function_name):
            setattr(MultiThreadedSubscriberNode, name, locals()[name])
    def topic_1_callback(self, msg):
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)  # 确保其他回调函数可以被调用
            self.get_logger().info(f'1')

    def topic_2_callback(self, msg):
        self.get_logger().info(f'Received message from topic_2: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    
    
    # 创建节点实例
    node = MultiThreadedSubscriberNode()
    
    # 使用MultiThreadedExecutor进行多线程处理
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
