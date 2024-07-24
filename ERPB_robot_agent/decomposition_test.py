import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from my_interfaces.srv import DictToBool, StringToBool, StringToDict
from my_interfaces.msg import Dictionary, KeyValue
import json
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class DecompositionTestNode(Node):

    def __init__(self, namespace):
        super().__init__('decomposition_test', namespace=namespace)
        
        self.callback_group = ReentrantCallbackGroup()

        # Create services
        self.task_manager_start_service = self.create_service(
            StringToBool,
            'task_manager_start_service',
            self.task_manager_start_callback,
            callback_group=self.callback_group
        )
        
        # self.bidding_evaluation_start_service = self.create_service(
        #     StringToBool,
        #     'bidding_evaluation_start_service',
        #     self.bidding_evaluation_start_callback,
        #     callback_group=self.callback_group
        # )
        
        self.black_board_add_service = self.create_service(
            StringToDict,
            '/black_board_add_service',
            self.black_board_add_callback,
            callback_group=self.callback_group
        )

        # Create publisher
        self.environment_info_publisher = self.create_publisher(String, '/environmental_information', 10)
        
        # Create client
        self.task_decomposition_client = self.create_client(StringToBool, 'task_decomposition_service')

        self.wait_for_services([self.task_decomposition_client])

        self.get_logger().info('Task Decomposition Test Node is ready.')

        self.timer = self.create_timer(1.0, self.publish_environment_info)
        self.environment_info_ready = False
        self.global_id_counter = 1  # Example counter for generating global IDs

    def wait_for_services(self, service_clients):
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)  # 确保其他回调函数可以被调用
            all_services_ready = True
            for service_client in service_clients:
                service_ready = service_client.wait_for_service(timeout_sec=1.0)
                if not service_ready:
                    self.get_logger().info(f'{service_client.srv_name} not available, waiting again...')
                    all_services_ready = False

            if all_services_ready:
                self.get_logger().info('All services are now available.')
                break

    def task_manager_start_callback(self, request, response):
        self.get_logger().info(f'Received task manager start request: {request.data}')
        task_data = json.loads(request.data)
        # Here you would add your task manager logic
        # Example: starting task execution
        response.success = True
        return response

    # def bidding_evaluation_start_callback(self, request, response):
    #     self.get_logger().info(f'Received bidding evaluation start request: {request.data}')
    #     task_list = json.loads(request.data)
    #     # Here you would add your bidding evaluation logic
    #     # Example: evaluating bids
    #     response.success = True
    #     return response

    def black_board_add_callback(self, request, response):
        self.get_logger().info(f'Received black board add request: {request.data}')
        data_packet = json.loads(request.data)
        task_list = data_packet['Task list']
        
        temp_to_global_id_map = {}
        for task in task_list:
            temp_id = task['Temp id']
            global_id = self.global_id_counter
            self.global_id_counter += 1
            temp_to_global_id_map[temp_id] = global_id
        
        # 创建一个 Dictionary 类型的实例列表
        dictionary_entries = Dictionary()
        for temp_id, global_id in temp_to_global_id_map.items():
            key_value_msg = KeyValue()
            key_value_msg.key = temp_id
            key_value_msg.value = str(global_id)
            dictionary_entries.entries.append(key_value_msg)
        
        # 将 dictionary_entries 赋值给 response.dict
        response.dict = dictionary_entries
        return response

    def publish_environment_info(self):
        msg = String()
        env_info = {
            "path_information": {
                "start_point": "M",
                "end_point": "B",  
                "status": "passable"
            },
            "current_robot_location": "M",
            "water_pump_location": "C",
            "drainage_pipe_location": "C",
            "ground_condition": "hard"
        }
        msg.data = json.dumps(env_info)
        self.environment_info_publisher.publish(msg)
        self.environment_info_ready = True
        self.get_logger().info('Published environmental information.')

    def send_task_decomposition_request(self):
        
        if not self.environment_info_ready:
            self.get_logger().info('Environmental information not ready, waiting...')
            return

        print(4567)

        request = StringToBool.Request()
        
        decompose_packet = {
            'My id': 'robot1',
            'Task content': 'Build a drainage pipe driven by a water pump from point A to point B, with the water pump installed at point B.',
            'Priority': 1,
            'My ability': 'Install the water pump',
            'From start flag': 'True',
            'Decomposed task type': 'operator',
            'Decomposed task id': '0'
        }
        
        request.data = json.dumps(decompose_packet)
        self.get_logger().info('Sending task decomposition request...')
        future = self.task_decomposition_client.call_async(request)
        future.add_done_callback(self.task_decomposition_response_callback)

    def task_decomposition_response_callback(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info('Task decomposition succeeded.')
        else:
            self.get_logger().info('Task decomposition failed.')


def main(args=None):
    rclpy.init(args=args)
    namespace = 'robot1'  # Example namespace, can be parameterized

    decomposition_test_node = DecompositionTestNode(namespace)

    executor = MultiThreadedExecutor()
    executor.add_node(decomposition_test_node)
    decomposition_test_node.publish_environment_info()

    try:
        decomposition_test_node.send_task_decomposition_request()
        executor.spin()
    finally:
        decomposition_test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
