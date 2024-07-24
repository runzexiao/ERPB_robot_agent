import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from my_interfaces.srv import StringToBool
import json

class TaskManagerNode(Node):
    def __init__(self, namespace):
        super().__init__('task_manager_node', namespace=namespace)
        self.callback_group = ReentrantCallbackGroup()

        self.task_management = {}

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

        # Timer
        self.timer = self.create_timer(5.0, self.timer_callback, callback_group=self.callback_group)

        self.get_logger().info(f'Task Manager Node of {namespace} is ready.')

    def task_manager_start_callback(self, request, response):
        try:
            request_dict = json.loads(request.data)
            task_list = request_dict['Task list']
            my_ability = request_dict['My ability']
            decomposed_task_id = request_dict['Decomposed task id']
            decomposed_task_type = request_dict['Decomposed task type']

            managed_task_info = []
            for task in task_list:
                task_info = {
                    'Task id': task['Task id'],
                    'Task type': task['Task type'],
                    'Content': task['Content'],
                    'Worker id': None,
                    'Task status': 'claimed' if task['Task type'] == 'My task' else 'published'
                }
                managed_task_info.append(task_info)

            self.task_management[decomposed_task_id] = managed_task_info

            self.get_logger().info(f'Task management updated: {self.task_management}')
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

    def timer_callback(self):
        try:
            for decomposed_task_id, managed_task_info in self.task_management.items():
                all_preliminary_collaborative_claimed = all(
                    task['Task status'] == 'claimed' for task in managed_task_info if task['Task type'] in ['preliminary task', 'collaborative task']
                )
                all_my_tasks_not_doing = all(
                    task['Task status'] != 'doing' for task in managed_task_info if task['Task type'] == 'My task'
                )

                if all_preliminary_collaborative_claimed and all_my_tasks_not_doing:
                    my_task = next((task for task in managed_task_info if task['Task type'] == 'My task'), None)
                    if my_task:
                        client = self.create_client(StringToBool, 'execute_node_start_service', callback_group=self.callback_group)
                        req = StringToBool.Request()
                        req.data = json.dumps({
                            'My ability': my_task['Content'],
                            'Decomposed task id': decomposed_task_id,
                            'Decomposed task type': 'My task',
                            'Task content': my_task['Content']
                        })
                        self.get_logger().info(f'Calling execute_node_start_service with {req.data}')
                        client.call_async(req)
                        my_task['Task status'] = 'doing'
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    namespace = 'robot1'
    node = TaskManagerNode(namespace)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
