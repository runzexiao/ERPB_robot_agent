import sys
import os

# 添加当前目录到搜索路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from my_interfaces.msg import Dictionary, KeyValue
from std_msgs.msg import String
from my_interfaces.srv import DictToBool, TaskStart, BlackBoardAdd
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import yaml
from task_decomposition_LLM_function import decompose_task
import json


class TaskDecompositionNode(Node):
    def __init__(self, namespace):
        super().__init__('task_decomposition_node', namespace=namespace)
        # 使用 ReentrantCallbackGroup 允许并发回调
        self.callback_group = ReentrantCallbackGroup()
        # Parameters
        self.pkg_name = "ERPB_robot_agent"
        self.params_file_name = "kcafe1_params.yaml"
        self.my_robot_id = namespace

        # Initialize environmental information
        self.environmental_information = {}
        self.environmental_info_ready = False

        # Load parameters
        self.load_parameters()

        # Subscribers
        self.environment_info_subscriber = self.create_subscription(
            String,
            '/environmental_information',
            self.environment_info_callback,
            10,
            callback_group=self.callback_group
        )

        # Service Clients
        self.task_manager_start_service_client = self.create_client(
            TaskStart,
            'task_manager_start_service',
            callback_group=self.callback_group
        )
        self.bidding_evaluation_start_service_client = self.create_client(
            TaskStart,
            'bidding_evaluation_start_service',
            callback_group=self.callback_group
        )
        self.black_board_add_service_client = self.create_client(
            BlackBoardAdd,
            '/black_board_add_service',
            callback_group=self.callback_group
        )

        # Service Server
        self.task_decomposition_service = self.create_service(
            DictToBool,
            'task_decomposition_service',
            self.task_decomposition_callback,
            callback_group=self.callback_group
        )

        # Wait for services to be available
        self.wait_for_services([
            self.task_manager_start_service_client,
            self.bidding_evaluation_start_service_client,
            self.black_board_add_service_client
        ])

        self.get_logger().info(f'Task Decomposition Node of {namespace} is ready.')

    def load_parameters(self):
        my_package_share_dir = get_package_share_directory(self.pkg_name)
        yaml_file_path = os.path.join(my_package_share_dir, 'config', self.params_file_name)
        with open(yaml_file_path, 'r') as file:
            config = yaml.safe_load(file)

        self.knowledge_input_list = config['Knowledge_input_list']
        self.get_logger().info(f'Knowledge input list: {self.knowledge_input_list}')

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

    
    def environment_info_callback(self, msg):
        self.environmental_information = json.loads(msg.data)
        self.environmental_info_ready = True
        self.get_logger().info(f'Updated environmental information: {self.environmental_information}')

    def task_decomposition_callback(self, request, response):
        self.get_logger().info("Starting task decomposition...")

        if not self.environmental_info_ready:
            self.get_logger().error('Environmental information not ready.')
            response.success = False
            return response

        # 从请求中提取信息
        request_dict = {entry.key: entry.value for entry in request.data.entries}
        boss_id = request_dict['Boss id']
        task_content = request_dict['Task content']
        priority = request_dict['Priority']
        my_ability = request_dict['My ability']
        from_start_flag = request_dict['From start flag']
        decomposed_task_type = request_dict['Decomposed task type']
        knowledge_of_decomposition = request_dict['Knowledge of decomposition']
        knowledge_template = request_dict['Knowledge template']

        self.get_logger().info(f'Task content: {task_content}')
        self.get_logger().info(f'My ability: {my_ability}')

        overall_task_dict = {"Content": task_content, "Priority": priority}

        # Call the task decomposition function
        data_packet, decomposed_task_list, suggestion_task_list = decompose_task(
            overall_task_dict,
            my_ability,
            self.environmental_information,
            self.knowledge_input_list,
            knowledge_of_decomposition,
            knowledge_template,
            self.my_robot_id,
            start_from_scratch=from_start_flag.lower() == 'true'
        )

        self.get_logger().info(f'Decomposed task list: {decomposed_task_list}')
        self.get_logger().info(f'Task dependencies: {data_packet["Local dependency graph"]}')

        # Request task_manager_start_service
        task_manager_req = TaskStart.Request()
        task_manager_req.data = self.dictionary_request_fill_in({
            'decomposed_task_list': decomposed_task_list,
            'knowledge_input_list': self.knowledge_input_list,
            'task_id': decomposed_task_type
        })
        self.task_manager_start_service_client.call_async(task_manager_req)

        # Request bidding_evaluation_start_service
        bidding_evaluation_req = TaskStart.Request()
        bidding_evaluation_req.data = self.dictionary_request_fill_in({
            'decomposed_task_list': decomposed_task_list,
            'knowledge_input_list': self.knowledge_input_list,
            'task_id': decomposed_task_type
        })
        self.bidding_evaluation_start_service_client.call_async(bidding_evaluation_req)

        # Request black_board_add_service
        black_board_add_req = BlackBoardAdd.Request()
        black_board_add_req.data = self.dictionary_request_fill_in({
            'decomposed_task_list': decomposed_task_list,
            'task_dependencies': data_packet["Local dependency graph"],
            'task_id': decomposed_task_type
        })
        black_board_add_resp = self.black_board_add_service_client.call(black_board_add_req)

        if black_board_add_resp.success:
            new_task_id = black_board_add_resp.task_id
            self.get_logger().info(f'Task successfully added to black board with new task ID: {new_task_id}')
        else:
            self.get_logger().error(f'Failed to add task to black board.')

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    namespace = 'robot1'  # Example namespace, can be参数化
    node = TaskDecompositionNode(namespace)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
