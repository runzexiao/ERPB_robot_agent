import sys
import os

# 添加当前目录到搜索路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from my_interfaces.srv import StringToBool, DictToBool
from bidding_LLM_function import compare_ability_with_task_fast
from ament_index_python.packages import get_package_share_directory
from my_interfaces.msg import Dictionaries, Dictionary, KeyValue
import yaml
import os

class OperatorTaskProcessingNode(Node):
    def __init__(self, namespace):
        super().__init__('operator_task_processing_node', namespace=namespace)
        # Parameters
        self.pkg_name = "ERPB_robot_agent"
        self.params_file_name = "kcafe1_params.yaml"
        self.my_robot_id = namespace

        # Load abilities
        self.load_abilities()

        # Definition of Service Server
        self.task_processing_start_service = self.create_service(
            StringToBool,
            'operator_task_processing_start_service',
            self.task_processing_start_callback
        )

        # Definition of Service Client
        self.task_decomposition_service_client = self.create_client(
            DictToBool,
            'task_decomposition_service'
        )

        self.wait_for_services([self.task_decomposition_service_client])
        self.get_logger().info(f'Operator Task Processing Node of {namespace} is ready.')

    def load_abilities(self):
        # 获取功能包的共享目录路径
        my_package_share_dir = get_package_share_directory(self.pkg_name)

        # 读取YAML文件
        yaml_file_path = os.path.join(my_package_share_dir, 'config', self.params_file_name)
        with open(yaml_file_path, 'r') as file:
            config = yaml.safe_load(file)

        # 从YAML文件中获取参数
        self.ability_list = config['Ability_list']
        self.get_logger().info(f'Ability list: {self.ability_list}')

    def wait_for_services(self, service_clients):
        while True:
            all_services_ready = True
            for service_client in service_clients:
                service_ready = service_client.wait_for_service(timeout_sec=1.0)
                if not service_ready:
                    self.get_logger().info(f'{service_client.srv_name} not available, waiting again...')
                    all_services_ready = False

            if all_services_ready:
                self.get_logger().info('All services are now available.')
                break

    def dictionary_request_fill_in(self, dict):
        dict_msg = Dictionary()
        for key, value in dict.items():
            key_value_msg = KeyValue()
            key_value_msg.key = key
            key_value_msg.value = str(value)
            dict_msg.entries.append(key_value_msg)

        return dict_msg

    def task_processing_start_callback(self, request, response):
        operator_task = request.data

        # Use compare_ability_with_task_fast to check task
        CAT_result_parser = compare_ability_with_task_fast(operator_task, self.ability_list)
        self.get_logger().info(f'Comparison result: {CAT_result_parser}')

        if CAT_result_parser:
            # Create the packet for task decomposition service when task is doable
            decompose_packet = {
                'Boss id': 0,
                'Task content': operator_task,
                'My ability': CAT_result_parser,
                'From strat flag': True,
                'Decomposed task type': 'None'
            }
        else:
            # Create the packet for task decomposition service when task is not doable
            decompose_packet = {
                'Boss id': 0,
                'Task content': operator_task,
                'My ability': 'None',
                'From strat flag': False,
                'Decomposed task type': 'None'
            }

        req = DictToBool.Request()
        req.data = self.dictionary_request_fill_in(decompose_packet)
        self.task_decomposition_service_client.call_async(req)
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    namespace = 'robot1'  # Example namespace, can be parameterized
    node = OperatorTaskProcessingNode(namespace)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
