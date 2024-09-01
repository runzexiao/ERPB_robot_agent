import sys
import os

# 添加当前目录到搜索路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import json
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from my_interfaces.srv import StringToBool, BoolStringToBool
from std_msgs.msg import String
from copy import deepcopy
from bidding_evaluation_LLM_function import select_robot_for_task

class BiddingEvaluationNode(Node):
    def __init__(self):
        super().__init__('bidding_evaluation_node')
        self.namespace = self.get_namespace().lstrip('/')
        self.task_dict = {}
        self.callback_group = ReentrantCallbackGroup()
        self.timer = None

        # 服务服务器
        self.bidding_evaluation_start_service = self.create_service(
            StringToBool,
            'bidding_evaluation_start_service',
            self.bidding_evaluation_start_callback,
            callback_group=self.callback_group
        )
        self.get_logger().info('Bidding Evaluation Node is ready.')

    def bidding_evaluation_start_callback(self, request, response):
        try:
            data_packet = json.loads(request.data)
            task_list = [d for d in data_packet['Task list'] if d['Task type'] != 'My task']
            decom_id = data_packet['decomposed task id']
            self.task_dict = self.generate_task_dict(task_list)
            self.create_bidding_update_services(self.task_dict)
            self.get_logger().info('Bidding Evaluation Node started.')

            # 启动定时器，等待10秒
            self.timer = self.create_timer(10.0, self.process_tasks_after_wait)

            response.success = True
        except Exception as e:
            self.get_logger().error(f'Failed to process request: {str(e)}')
            response.success = False
        return response

    def generate_task_dict(self, task_list):
        task_dict = {}
        for task in task_list:
            task_id = task['Task id']
            task_dict[task_id] = {
                'bidding': [],
                'task description': task['Content'],
                'task status': 'published'
            }
        return task_dict

    def create_bidding_update_services(self, task_dict):
        for task_id in task_dict:
            service_name = f'task_{task_id}/bidding_evaluation_update_service'
            self.create_service(
                StringToBool,
                service_name,
                self.bidding_evaluation_update_callback,
                callback_group=self.callback_group
            )
            self.get_logger().info(f'Created service: {service_name}')

    def bidding_evaluation_update_callback(self, request, response):
        bidding_packet = json.loads(request.data)
        task_id = bidding_packet['Task id']
        robot_id = bidding_packet['Robot id']

        if self.task_dict[task_id]['task status'] == 'published':
            self.task_dict[task_id]['bidding'].append(bidding_packet)
        elif self.task_dict[task_id]['task status'] == 'claimed':
            self.bidding_result_input_service_wall_client = self.create_client(
                BoolStringToBool,
                f'/{robot_id}/bidding_result_input_service',
                callback_group=self.callback_group
            )
            req = BoolStringToBool.Request()
            req.bool_data = False
            req.string_data = task_id
            self.bidding_result_input_service_wall_client.call_async(req)
        
        response.success = True
        return response

    def process_tasks_after_wait(self):
        self.timer.cancel()
        results = {}
        task_dict_now = deepcopy(self.task_dict)
        task_dict_published_now = {task_id: task_content for task_id, task_content in task_dict_now.items() if task_content['task status'] == 'published'}
        
        for task_id, task_content in task_dict_published_now.items():
            bidding_list = task_content['bidding']
            if bidding_list:
                result = select_robot_for_task(task_content['task description'], bidding_list)
            else:
                result = False
            
            results[task_id] = result
            if result:
                self.task_dict[task_id]['task status'] = 'claimed'
                self.get_logger().info(f'Task {task_id} claimed by robot {result}')
                self.notify_robots(task_id, result, bidding_list)
            else:
                self.reject_task(task_id, bidding_list)

        rclpy.spin_once(self, timeout_sec=0.5)

        task_dict_now_2 = deepcopy(self.task_dict)
        for task_id, new_task_content in task_dict_now_2.items():
            if task_id in task_dict_now:
                old_list = task_dict_now[task_id].get('bidding', [])
                new_list = new_task_content.get('bidding', [])
                updated_list = [item for item in new_list if item not in old_list]
                self.task_dict[task_id]['bidding'] = updated_list
        
        if not all(results.values()):
            self.timer = self.create_timer(10.0, self.process_tasks_after_wait)
        else:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.cleanup_remaining_biddings()

    def notify_robots(self, task_id, selected_robot_id, bidding_list):
        self.bidding_result_input_service_client = self.create_client(
            BoolStringToBool,
            f'/{selected_robot_id}/bidding_result_input_service',
            callback_group=self.callback_group
        )
        req = BoolStringToBool.Request()
        req.bool_data = True
        req.string_data = task_id
        self.bidding_result_input_service_client.call_async(req)

        others = [robot['Robot id'] for robot in bidding_list if robot['Robot id'] != selected_robot_id]
        for robot_id in others:
            self.bidding_result_input_service_client = self.create_client(
                BoolStringToBool,
                f'/{robot_id}/bidding_result_input_service',
                callback_group=self.callback_group
            )
            req = BoolStringToBool.Request()
            req.bool_data = False
            req.string_data = task_id
            self.bidding_result_input_service_client.call_async(req)

    def reject_task(self, task_id, bidding_list):
        self.get_logger().warn(f'Task {task_id} has no suitable bidding, retrying in 10 seconds...')
        robot_ids = [robot['Robot id'] for robot in bidding_list]
        for robot_id in robot_ids:
            self.bidding_result_input_service_client = self.create_client(
                BoolStringToBool,
                f'/{robot_id}/bidding_result_input_service',
                callback_group=self.callback_group
            )
            req = BoolStringToBool.Request()
            req.bool_data = False
            req.string_data = task_id
            self.bidding_result_input_service_client.call_async(req)

    def cleanup_remaining_biddings(self):
        task_dict_now_3 = deepcopy(self.task_dict)
        for task_id, task_content in task_dict_now_3.items():
            if task_content['bidding']:
                for bidding in task_content['bidding']:
                    self.bidding_result_input_service_client = self.create_client(
                        BoolStringToBool,
                        f'/{bidding["Robot id"]}/bidding_result_input_service',
                        callback_group=self.callback_group
                    )
                    req = BoolStringToBool.Request()
                    req.bool_data = False
                    req.string_data = task_id
                    self.bidding_result_input_service_client.call_async(req)
        self.get_logger().info('All tasks have been claimed')

def main(args=None):
    rclpy.init(args=args)
    node = BiddingEvaluationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
