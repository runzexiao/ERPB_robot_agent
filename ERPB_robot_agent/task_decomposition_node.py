import sys
import os

# 添加当前目录到搜索路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from my_interfaces.msg import Dictionary, KeyValue
from std_msgs.msg import String
from my_interfaces.srv import DictToBool, StringToBool, StringToDict, StringToString
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import yaml
from task_decomposition_LLM_function import decompose_task
from same_test_LLM_function import compare_two_text_content
import json


class TaskDecompositionNode(Node):
    def __init__(self):
        super().__init__('task_decomposition_node')
        # 使用 ReentrantCallbackGroup 允许并发回调
        self.callback_group = ReentrantCallbackGroup()
        # Parameters
        self.pkg_name = "ERPB_robot_agent"
        self.my_robot_id = self.get_namespace().lstrip('/')
        self.params_file_name = self.my_robot_id  + "_params.yaml"
        

        # Initialize environmental information
        self.environmental_information = {}
        self.environmental_info_ready = False

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
            StringToBool,
            'task_manager_start_service',
            callback_group=self.callback_group
        )
        self.bidding_evaluation_start_service_client = self.create_client(
            StringToBool,
            'bidding_evaluation_start_service',
            callback_group=self.callback_group
        )
        self.broadcaster_add_service_client = self.create_client(
            StringToString,
            '/broadcaster_add_service',
            callback_group=self.callback_group
        )

        # Service Server
        self.task_decomposition_service = self.create_service(
            StringToBool,
            'task_decomposition_service',
            self.task_decomposition_callback,
            callback_group=self.callback_group
        )

        # Wait for services to be available
        self.wait_for_services([
            self.task_manager_start_service_client,
            self.bidding_evaluation_start_service_client,
            self.broadcaster_add_service_client
        ])

        self.get_logger().info(f'Task Decomposition Node of {self.my_robot_id} is ready.')

    def load_parameters(self, ability):
        ability_slash = ability.replace(' ', '_')
        my_package_share_dir = get_package_share_directory(self.pkg_name)
        yaml_file_path = os.path.join(my_package_share_dir, 'config', self.params_file_name)
        with open(yaml_file_path, 'r') as file:
            config = yaml.safe_load(file)

        self.knowledge_needed_input = config[ability_slash]['knowledge_needed_input']
        self.knowledge_of_decomposition = config['knowledge_of_decomposition']
        self.knowledge_template = config[ability_slash]['knowledge_template']
        self.get_logger().info(f'Knowledge input list: {self.knowledge_needed_input}')

    def create_data_packet_for_broadcaster(self, task_list, robot_id, decompose_task_id):
        temp_id_map = {}
        local_dependency_graph = {}
        current_temp_id = 1

        # Assign temporary IDs
        for task in task_list:
            temp_id = str(current_temp_id)
            current_temp_id += 1
            task['Temp id'] = temp_id
            task['Boss id'] = robot_id
            task['Worker id'] = None if task['Task type'] != 'My task' else robot_id
            task['Decomposed from task id'] = decompose_task_id
            task['Shadow boss'] = []
            temp_id_map[task['Content']] = temp_id

        # Generate dependency graph based on task type
        for task in task_list:
            temp_id = task['Temp id']
            task_type = task['Task type']
            dependencies = []

            if task_type == 'My task':
                dependencies = [temp_id_map[t['Content']] for t in task_list if t['Task type'] == 'preliminary task']
            elif task_type == 'subsequent task':
                dependencies = [temp_id_map[t['Content']] for t in task_list if t['Task type'] == 'My task']

            local_dependency_graph[temp_id] = dependencies

        data_packet = {
            'Task list': task_list,
            'Local dependency graph': local_dependency_graph,
            'Decomposed from task id': decompose_task_id
        }
        return data_packet
    
    def create_data_packet_for_manager(self, task_list, temp_to_global_id_map, my_ability, decomposed_task_id, decomposed_task_type, Father_decomposed_from_task_id, boss_id, Shadow_boss):
            
        for task in task_list:
            temp_id = task.get("Temp id")
            if temp_id in temp_to_global_id_map:
                task["Task id"] = temp_to_global_id_map[temp_id]
                del task["Temp id"]
        
        data_packet = {
            'Task list': task_list,
            'My ability': my_ability,
            'Decomposed task id': decomposed_task_id,
            'Decomposed task type': decomposed_task_type,
            'Father decomposed from task id':Father_decomposed_from_task_id,
            'Boss id': boss_id,
            'Shadow boss': Shadow_boss
        }
        return data_packet

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
        #self.get_logger().info(f'Updated environmental information: {self.environmental_information}')
    
    def starts_with_prefix(self, main_string, prefix):
        return main_string.startswith(prefix)

    def task_decomposition_callback(self, request, response):
        try:
            self.get_logger().info("Starting task decomposition...")

            # if not self.environmental_info_ready:
            #     self.get_logger().info('Environmental information not ready.')
            #     # response.success = False
            #     # return response
            
            while not self.environmental_info_ready:
                rclpy.spin_once(self, timeout_sec=0.9)  # 确保其他回调函数可以被调用
                self.get_logger().info('Environmental information not ready.')

            

            # 从请求中提取信息
            # request_dict = {entry.key: entry.value for entry in request.data.entries}
            request_dict = json.loads(request.data)
            my_id = request_dict['My id']
            task_content = request_dict['Task content']
            priority = request_dict['Priority']
            my_ability = request_dict['My ability']
            from_start_flag = request_dict['From start flag']
            decomposed_task_type = request_dict['Decomposed task type']
            decomposed_task_id = request_dict['Decomposed task id']
            Father_decomposed_from_task_id = request_dict['Father decomposed from task id']
            boss_id = request_dict['Boss id']
            Shadow_boss = request_dict['Shadow boss']
            
            if my_ability != 'None':
                self.load_parameters(my_ability)
            else:
                self.knowledge_needed_input = 'Nothing'
                self.knowledge_template = 'Nothing'
            

            self.get_logger().info(f'Task content: {task_content}')
            self.get_logger().info(f'My ability: {my_ability}')

            overall_task_dict = {"Content": task_content, "Priority": priority}
            # rclpy.spin_once(self, timeout_sec=0.1)  # 确保其他回调函数可以被调用
            # Call the task decomposition function
            decomposed_task_list, suggestion_task_list = decompose_task(
                my_id,
                overall_task_dict,
                my_ability,
                self.environmental_information,
                self.knowledge_needed_input,
                self.knowledge_template,
                from_start_flag
            )

            ##Only for listener cop
            if decomposed_task_type == 'collaborative task':
                decomposed_task_list = [task for task in decomposed_task_list if task['Task type'] != 'collaborative task']
            if decomposed_task_type == 'subsequent task':
                self.get_logger().info('Fall in subsequent task branch')
                preliminary_task_list = [task for task in decomposed_task_list if task['Task type'] == 'preliminary task']
                qury_id = Father_decomposed_from_task_id
                self.get_logger().info(f'qury_id: {qury_id}')
                gg_client = self.create_client(StringToString, f'/{boss_id}/manager_query_service')
                req={'Decomposed task id': qury_id, 'qkey': 'Task type', 'qkeyvalue': 'My task', 'akey': 'Content'}
                gg_request = StringToString.Request()
                gg_request.request_data = json.dumps(req)
                self.get_logger().info(f' request.request_data: {gg_request.request_data}')
                gg_response = gg_client.call(gg_request)
                father_task_content = gg_response.response_data
                gg_client.destroy()
                for task in preliminary_task_list:
                    if compare_two_text_content(task['Content'], father_task_content):
                        self.get_logger().info(f'Found the same task: {task}')
                        break
                else:
                    self.get_logger().info(f'No same task found, add father task {father_task_content} to decomposed task list')
                    decomposed_task_list.append({'Task type': 'preliminary task', 'Content': father_task_content, 'Priority': priority +1})
                    
            
            self.get_logger().info(f'Decomposed task list: {decomposed_task_list}')
            data_packet_for_broadcaster = self.create_data_packet_for_broadcaster(decomposed_task_list, self.my_robot_id, decomposed_task_id)
            # Request broadcaster_add_service
            broadcaster_add_req = StringToString.Request()
            broadcaster_add_req.request_data = json.dumps(data_packet_for_broadcaster)
            broadcast_answer_unloaded = self.broadcaster_add_service_client.call(broadcaster_add_req)
            broadcast_answer = json.loads(broadcast_answer_unloaded.response_data)
            # broadcast_answer = {'exsiting task list': [], 'temp to global map':{}}
            
            temp_to_global_id_map = broadcast_answer['temp to global map']
            exsiting_task_list = broadcast_answer['exsiting task list']

            data_packet_for_manager = self.create_data_packet_for_manager(data_packet_for_broadcaster['Task list'], temp_to_global_id_map, my_ability, decomposed_task_id, decomposed_task_type, Father_decomposed_from_task_id, boss_id, Shadow_boss)
    
            # Request task_manager_start_service
            task_manager_req = StringToBool.Request()
            task_manager_req.data = json.dumps(data_packet_for_manager)
            self.get_logger().info(f'task_manager_req:{task_manager_req.data}')
            self.task_manager_start_service_client.call_async(task_manager_req)

            # Request bidding_evaluation_start_service
            bidding_evaluation_req = StringToBool.Request()
            bidding_evaluation_req.data = json.dumps({'Task list': [task for task in data_packet_for_manager['Task list'] if task['Task id'] not in exsiting_task_list], 'decomposed task id': decomposed_task_id})
            self.get_logger().info(f'bidding_evaluation_req:{bidding_evaluation_req.data}')
            self.bidding_evaluation_start_service_client.call_async(bidding_evaluation_req)
            response.success = True
        except ValueError as e:
            self.get_logger().error(f"发生了 ValueError 错误: {e}")
            # 处理异常或提供默认响应
        except Exception as e:
            self.get_logger().error(f"发生了意外错误: {e}")
        return response



def main(args=None):
    rclpy.init(args=args)
    node = TaskDecompositionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
