import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from my_interfaces.srv import StringToBool, StringToString
from std_srvs.srv import Trigger
import json

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        self.callback_group = ReentrantCallbackGroup()
        self.my_robot_id = self.get_namespace().lstrip('/')
        self.task_management = {}
        self.shadow_boss_list = {}
        self.ability_for_my_task_list = {}
        self.decomposed_task_type_list = {}
        self.boss_id_list = {}
        self.Father_decomposed_from_task_id_list = {} 
        self.exe_lock = False
        self.timer = None
        self.status_sent = {}  # 记录已发送的状态更新
        self.collaborative_robots_info_list = {}  # 存储collaborative basic info

        # Services
        self.task_manager_start_service = self.create_service(
            StringToBool,
            'task_manager_start_service',
            self.task_manager_start_callback,
            callback_group=self.callback_group
        )
        self.managed_task_info_update_service = self.create_service(
            StringToBool,
            'managed_task_info_update',
            self.managed_task_info_update_callback,
            callback_group=self.callback_group
        )

        self.manager_shadow_boss_append_service = self.create_service(
            StringToBool,
            'manager_shadow_boss_append',
            self.manager_shadow_boss_append_callback,
            callback_group=self.callback_group
        )

        self.exe_unlock_service = self.create_service(
            Trigger,
            'exe_unlock',
            self.exe_unlock_callback,
            callback_group=self.callback_group
        )
        self.cop_basic_info_add_service = self.create_service(
            StringToBool,
            'cop_basic_info_add',
            self.cop_basic_info_add_callback,
            callback_group=self.callback_group
        )
        self.manager_query_service = self.create_service(
            StringToString,
            'manager_query_service',
            self.manager_query_service_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info(f'Task Manager Node of {self.my_robot_id} is ready.')

    def exe_unlock_callback(self, request, response):
        self.exe_lock = False
        self.get_logger().info(f'Execution unlocked')
        response.success = True
        response.message = 'Execution unlocked'
        return response
    
    def manager_query_service_callback(self, request, response):
        request_dict = json.loads(request.request_data)
        Decom_id = request_dict['Decomposed task id']
        qkey = request_dict['qkey'] # 'Task Type'
        qkeyvalue = request_dict['qkeyvalue'] # 'My task'
        akey = request_dict['akey'] # 'Content'

        my_task_content = next((task[akey] for task in self.task_management[Decom_id] if task[qkey] == qkeyvalue), None)
        response.response_data = my_task_content
        return response

    def task_manager_start_callback(self, request, response):
        try:
            request_dict = json.loads(request.data)
            task_list = request_dict['Task list']
            my_ability = request_dict['My ability']
            decomposed_task_id = request_dict['Decomposed task id']
            decomposed_task_type = request_dict['Decomposed task type']
            Father_decomposed_from_task_id = request_dict['Father decomposed from task id']
            boss_id = request_dict['Boss id']
            Shadow_boss = request_dict['Shadow boss']
            

            managed_task_info = []
            for task in task_list:
                task_info = {
                    'Task id': task['Task id'],
                    'Task type': task['Task type'],
                    'Content': task['Content'],
                    'Worker id': task['Worker id'],
                    'Task status': 'Claimed' if task['Task type'] == 'My task' else 'Published'
                }
                managed_task_info.append(task_info)

            self.task_management[decomposed_task_id] = managed_task_info
            self.ability_for_my_task_list[decomposed_task_id] = my_ability
            self.decomposed_task_type_list[decomposed_task_id] = decomposed_task_type
            self.boss_id_list[decomposed_task_id] = boss_id
            self.Father_decomposed_from_task_id_list[decomposed_task_id] = Father_decomposed_from_task_id
            self.shadow_boss_list[decomposed_task_id] = Shadow_boss
            self.collaborative_robots_info_list[decomposed_task_id] = []

            self.status_sent[decomposed_task_id] = {}  # 初始化状态记录

            self.get_logger().info(f'Task management updated: {self.task_management}')

            # Timer
            if self.timer is None:
                self.timer = self.create_timer(5.0, self.timer_callback, callback_group=self.callback_group)

            response.success = True
        except Exception as e:
            self.get_logger().error(f'Error in task_manager_start_callback: {e}')
            response.success = False

        return response

    def managed_task_info_update_callback(self, request, response):
        try:
            request_dict = json.loads(request.data)
            decomposed_task_id = request_dict['Decomposed task id']
            task_id = request_dict['Task id']
            key = request_dict['key']
            keyvalue = request_dict['keyvalue']

            for task in self.task_management[decomposed_task_id]:
                if task['Task id'] == task_id:
                    task[key] = keyvalue
                    self.get_logger().info(f'Task {task_id} updated with {key}: {keyvalue}')
                    response.success = True
                    return response

            self.get_logger().error(f'Task {task_id} not found in decomposed task {decomposed_task_id}')
            response.success = False
        except Exception as e:
            self.get_logger().error(f'Error in managed_task_info_update_callback: {e}')
            response.success = False

        return response
    
    def manager_shadow_boss_append_callback(self, request, response):
        try:
            request_dict = json.loads(request.data)
            decomposed_task_id = request_dict['Decomposed task id']
            shadow_boss = request_dict['Shadow boss']

            self.shadow_boss_list[decomposed_task_id].append(shadow_boss)
            self.get_logger().info(f'Shadow boss {shadow_boss} appended to decomposed task {decomposed_task_id}')
            response.success = True
            return response
      
        except Exception as e:
            self.get_logger().error(f'Error in managed_task_info_update_append_callback: {e}')
            response.success = False
            return response

        

    def cop_basic_info_add_callback(self, request, response):
        try:
            request_dict = json.loads(request.data)
            robot_id = request_dict['robot id']
            task_id = request_dict['task id']
            task_description = request_dict['task description']
            my_function_description = request_dict['my function description']
            decomposed_task_id = request_dict['decomposed task id']
           

            info = {
                'collaborative_robots_basic_info': {
                    'robot id': robot_id,
                    'task id': task_id,
                    'task description': task_description
                },
                'cop_function_description': '{{' + f'"robot id": {robot_id}' + f', "function_description": {my_function_description}' + '}}'
            }

            self.get_logger().info(f'CCCCCCCCollaborative basic info received: {info["cop_function_description"]}')

            if decomposed_task_id not in self.collaborative_robots_info_list:
                self.collaborative_robots_info_list[decomposed_task_id] = []

            self.collaborative_robots_info_list[decomposed_task_id].append(info)
            # self.get_logger().info(f'Collaborative basic info updated: {self.collaborative_robots_info_list}')

            response.success = True
        except Exception as e:
            self.get_logger().error(f'Error in cop_basic_info_add_callback: {e}')
            response.success = False

        return response

    def timer_callback(self):
  
        try:
            for decomposed_task_id, managed_task_info in self.task_management.items():
                all_preliminary_done = all(
                    task['Task status'] == 'Done' for task in managed_task_info if task['Task type'] in ['preliminary task']
                )
                all_collaborative_claimed = all(
                    task['Task status'] == 'prepared' for task in managed_task_info if task['Task type'] in ['collaborative task']
                )
                # self.get_logger().info(f'All collaborative tasks claimed: {all_collaborative_claimed}')
                all_my_tasks_not_doing = all(task['Task status'] != 'Doing' for id, tasklist in self.task_management.items() if id != decomposed_task_id for task in tasklist if task['Task type'] == 'My task')
                if all_preliminary_done and all_collaborative_claimed and all_my_tasks_not_doing and (not self.exe_lock):
                    self.get_logger().info(f'Ready for calling execute_node_start_service.')
                    my_task = next((task for task in managed_task_info if task['Task type'] == 'My task'), None)
                    if my_task:
                        client = self.create_client(StringToBool, 'execute_node_start_service', callback_group=self.callback_group)
                        req = StringToBool.Request()
                        req.data = json.dumps({
                            'My ability': self.ability_for_my_task_list[decomposed_task_id],
                            'Decomposed task id': decomposed_task_id,
                            'Decomposed task type': self.decomposed_task_type_list[decomposed_task_id],
                            'Task content': my_task['Content'],
                            'Father decomposed from task id': self.Father_decomposed_from_task_id_list[decomposed_task_id],
                            'Boss id': self.boss_id_list[decomposed_task_id],
                            'My task id': my_task['Task id'],
                            'Collaborative robots info list': self.collaborative_robots_info_list[decomposed_task_id]
                        })
                        # self.get_logger().info(f'Calling execute_node_start_service with {req.data}')
                        self.get_logger().info(f'XXXXXXXcop_list before exe with decomposed task id {decomposed_task_id}: {self.collaborative_robots_info_list[decomposed_task_id]}')
                        client.call_async(req)
                        self.exe_lock = True
                        self.get_logger().info(f'Execution node is locked.')
                        break

                # 检查并发送状态更新请求
                current_status = 'Doing'
                if (any(task['Task status'] == 'Doing' for task in managed_task_info if task['Task type'] in ['My task', 'independent task', 'collaborative task'])
                    and self.status_sent[decomposed_task_id].get(current_status) != True):
                    boss_id = self.boss_id_list[decomposed_task_id]
                    self.get_logger().info(f'XXXXYYYYXXXYYYYYboss_id_list: {self.boss_id_list}')
                    client = self.create_client(StringToBool, f'/{boss_id}/managed_task_info_update', callback_group=self.callback_group)
                    req = StringToBool.Request()
                    req.data = json.dumps({
                        'Decomposed task id': self.Father_decomposed_from_task_id_list[decomposed_task_id],
                        'Task id': decomposed_task_id,
                        'key': 'Task status',
                        'keyvalue': 'Doing'
                    })
                    self.get_logger().info(f'Updating boss {boss_id} with task {decomposed_task_id} status to "Doing"')
                    client.call_async(req)

                    shadow_list = self.shadow_boss_list[decomposed_task_id]
                    if shadow_list:
                        for shadow in shadow_list:
                            shadow_boss_id = shadow['Shadow boss id']
                            shadow_decomposed_id = shadow['Shadow decomposed id']
                            shadow_task_id = shadow['Shadow task id']
                            client1 = self.create_client(StringToBool, f'/{shadow_boss_id}/managed_task_info_update', callback_group=self.callback_group)
                            req1 = StringToBool.Request()
                            req1.data = json.dumps({
                                'Decomposed task id': shadow_decomposed_id,
                                'Task id': shadow_task_id,
                                'key': 'Task status',
                                'keyvalue': 'Doing'
                            })
                            self.get_logger().info(f'Updating boss {shadow_boss_id} with task {shadow_task_id} status to "Doing"')
                            client1.call_async(req1)

                    self.status_sent[decomposed_task_id][current_status] = True

                current_status = 'Done'
                if (all(task['Task status'] == 'Done' for task in managed_task_info if task['Task type'] in ['My task', 'independent task', 'collaborative task'])
                    and self.status_sent[decomposed_task_id].get(current_status) != True):
                    boss_id = self.boss_id_list[decomposed_task_id]
                    client = self.create_client(StringToBool, f'/{boss_id}/managed_task_info_update', callback_group=self.callback_group)
                    req = StringToBool.Request()
                    req.data = json.dumps({
                        'Decomposed task id': self.Father_decomposed_from_task_id_list[decomposed_task_id],
                        'Task id': decomposed_task_id,
                        'key': 'Task status',
                        'keyvalue': 'Done'
                    })
                    self.get_logger().info(f'Updating boss {boss_id} with task {decomposed_task_id} status to "Done"')
                    client.call_async(req)
                    
                    shadow_list = self.shadow_boss_list[decomposed_task_id]
                    if shadow_list:
                        for shadow in shadow_list:
                            shadow_boss_id = shadow['Shadow boss id']
                            shadow_decomposed_id = shadow['Shadow decomposed id']
                            shadow_task_id = shadow['Shadow task id']
                            client1 = self.create_client(StringToBool, f'/{shadow_boss_id}/managed_task_info_update', callback_group=self.callback_group)
                            req1 = StringToBool.Request()
                            req1.data = json.dumps({
                                'Decomposed task id': shadow_decomposed_id,
                                'Task id': shadow_task_id,
                                'key': 'Task status',
                                'keyvalue': 'Done'
                            })
                            self.get_logger().info(f'Updating boss {shadow_boss_id} with task {shadow_task_id} status to "Doing"')
                            client1.call_async(req1)


                    self.status_sent[decomposed_task_id][current_status] = True

                current_status = 'prepared'
                if (all(task['Task status'] == 'prepared' for task in managed_task_info if task['Task type'] in ['My task', 'collaborative task'])
                    and self.status_sent[decomposed_task_id].get(current_status) != True):
                    boss_id = self.boss_id_list[decomposed_task_id]
                    client = self.create_client(StringToBool, f'/{boss_id}/managed_task_info_update', callback_group=self.callback_group)
                    req = StringToBool.Request()
                    req.data = json.dumps({
                        'Decomposed task id': self.Father_decomposed_from_task_id_list[decomposed_task_id],
                        'Task id': decomposed_task_id,
                        'key': 'Task status',
                        'keyvalue': 'prepared'
                    })
                    self.get_logger().info(f'Updating boss {boss_id} with task {decomposed_task_id} status to "prepared"')
                    client.call_async(req)

                    shadow_list = self.shadow_boss_list[decomposed_task_id]
                    if shadow_list:
                        for shadow in shadow_list:
                            shadow_boss_id = shadow['Shadow boss id']
                            shadow_decomposed_id = shadow['Shadow decomposed id']
                            shadow_task_id = shadow['Shadow task id']
                            client1 = self.create_client(StringToBool, f'/{shadow_boss_id}/managed_task_info_update', callback_group=self.callback_group)
                            req1 = StringToBool.Request()
                            req1.data = json.dumps({
                                'Decomposed task id': shadow_decomposed_id,
                                'Task id': shadow_task_id,
                                'key': 'Task status',
                                'keyvalue': 'prepared'
                            })
                            self.get_logger().info(f'Updating boss {shadow_boss_id} with task {shadow_task_id} status to "Doing"')
                            client1.call_async(req1)

                    self.status_sent[decomposed_task_id][current_status] = True
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
