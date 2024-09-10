import sys
import os

# 添加当前目录到搜索路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
from my_interfaces.srv import StringToBool
from std_srvs.srv import Trigger
import json
from execute_LLM_function import generate_function_trans_result
from ament_index_python.packages import get_package_share_directory
import yaml
import math
from threading import Event


class ExecuteNode(Node):
    def __init__(self):
        super().__init__('execute_node')
        self.callback_group = ReentrantCallbackGroup()
        self.robot_id = self.get_namespace().lstrip('/')
        self.pkg_name = "ERPB_robot_agent"
        self.params_file_name = self.robot_id + "_params.yaml"
        self.pos = Pose()
        # Services
        self.instruction_listener_service = self.create_service(
            StringToBool,
            'instruction_listener_service',
            self.instruction_listener_callback,
            callback_group=self.callback_group
        )

        self.execute_node_start_service = self.create_service(
            StringToBool,
            'execute_node_start_service',
            self.execute_node_start_callback,
            callback_group=self.callback_group
        )

        # Defination of broadcaster_update_service_client
        self.broadcaster_update_service_client = self.create_client(
            StringToBool, 
            '/broadcaster_task_info_update',
            callback_group=self.callback_group
        )


        # Initialize environmental information
        self.environment_info = {}
        self.environmental_info_ready = False

        # Subscribers
        self.environment_info_subscriber = self.create_subscription(
            String,
            '/environmental_information',
            self.environment_info_callback,
            10,
            callback_group=self.callback_group
        )

        self.get_logger().info(f'Execute Node of {self.robot_id} is ready.')

    def environment_info_callback(self, msg):
        self.environment_info = json.loads(msg.data)
        self.current_position = self.environment_info['robot_location'][self.robot_id]
        # self.get_logger().info(f'current_position: {self.current_position}')
        self.environmental_info_ready = True
        #self.get_logger().info(f'Updated environmental information: {self.environment_info}')
    def load_parameters(self, ability):
        self.get_logger().info('Load Params!')
        ability_slash = ability.replace(' ', '_')
        my_package_share_dir = get_package_share_directory(self.pkg_name)
        yaml_file_path = os.path.join(my_package_share_dir, 'config', self.params_file_name)
        with open(yaml_file_path, 'r') as file:
            config = yaml.safe_load(file)

        self.get_logger().info('Load over!')

        self.execute_needed_input = config[ability_slash]['execute_needed_input']#Dictionary T
        self.execute_input_variable_list = config[ability_slash]['execute_input_variable_list']#variable_list

        self.execute_process = config[ability_slash]['execute_process']#T1

        self.my_function_description = config[ability_slash]['my_function_description']#T2


        self.my_function_defination = config[ability_slash]['my_function_defination']#T3

        self.my_function_name = config[ability_slash]['my_function_name']


        self.get_logger().info(f'my_function_defination{self.my_function_defination}')
        exec(self.my_function_defination)  # 动态加载方法到类中

        # 将定义的函数绑定到 MyRobot 类
        for name in eval(self.my_function_name):
            setattr(ExecuteNode, name, locals()[name])

    

    def instruction_listener_callback(self, request, response):
        self.get_logger().info(f'XXRZ instruction_listener_callback: {request.data}')
        request_dict = json.loads(request.data)
        function_name = request_dict['function name']
        function_input = request_dict['function input']
        lag = 'self.' + function_name + '(*' + str(function_input) +')'
        self.get_logger().info(f'execute from listener XRZ lag: {lag}')
        eval(lag)
        self.get_logger().info(f'XXRZ after eval:{lag}')
        response.success = True
        self.get_logger().info(f'XXRZ response.success :{response.success}')
        return response



    def execute_node_start_callback(self, request, response):

        while not self.environmental_info_ready:
            rclpy.spin_once(self, timeout_sec=0.9)  # 确保其他回调函数可以被调用
            self.get_logger().info('Environmental information not ready.')

        try:
            request_dict = json.loads(request.data)
            my_ability = request_dict['My ability']
            decomposed_task_id = request_dict['Decomposed task id']
            decomposed_task_type = request_dict['Decomposed task type']
            task_content = request_dict['Task content']
            Father_decomposed_from_task_id = request_dict['Father decomposed from task id']
            boss_id = request_dict['Boss id']
            my_task_id = request_dict['My task id']

            ######################
            collaborative_robots_info_list = request_dict['Collaborative robots info list']
            collaborative_robots_basic_info = [a['collaborative_robots_basic_info'] for a in collaborative_robots_info_list]
            cops_function_description = [a['cop_function_description'] for a in collaborative_robots_info_list]
            cops_function_description = '[' + ",".join(cops_function_description) + ']'
         
            self.get_logger().info('Before loading params.')
            self.load_parameters(my_ability)

            self.get_logger().info('After loading params.')
            # 判断'Decomposed task type'的值
            if decomposed_task_type != 'collaborative task':
                self.update_task_status(decomposed_task_id, my_task_id, 'Doing')
                for basic_info in collaborative_robots_basic_info:
                    self.update_task_status(decomposed_task_id, basic_info['task id'], 'Doing') 
                    

                # 调用generate_function_trans_result函数
                
                self.get_logger().info(f'task_content:{type(task_content)},self.robot_id:{type(self.robot_id)},collaborative_robots_basic_info:{type(collaborative_robots_basic_info)},self.environment_info:{type(self.environment_info)},self.execute_needed_input:{type(self.execute_needed_input)},self.execute_input_variable_list:{type(self.execute_input_variable_list)},self.execute_process:{type(self.execute_process)},self.my_function_description:{type(self.my_function_description)},cops_function_description:{type(cops_function_description)}')
                self.get_logger().info(f'QQQWWWW{collaborative_robots_basic_info}')
                self.get_logger().info(f'QQQWWWW{cops_function_description}')
                # self.get_logger().info(f'task_content:{task_content},self.robot_id:{self.robot_id},collaborative_robots_basic_info:{collaborative_robots_basic_info},self.environment_info:{self.environment_info},self.execute_needed_input:{self.execute_needed_input},self.execute_input_variable_list:{self.execute_input_variable_list},self.execute_process:{self.execute_process},self.my_function_description:{self.my_function_description},cops_function_description:{cops_function_description}')
                function_list = generate_function_trans_result(task_content, self.robot_id, collaborative_robots_basic_info, self.environment_info, self.execute_needed_input, eval(self.execute_input_variable_list), self.execute_process, self.my_function_description, cops_function_description)
                self.get_logger().info(f'function_list: {function_list}')    
                # 执行function_list的函数
                for function in function_list:
                    self.get_logger().info(f'Execute function XRZ: {function}')
                    eval(function)
                   
           

                self.update_task_status(decomposed_task_id, my_task_id, 'Done')
                for basic_info in collaborative_robots_basic_info:
                    self.update_task_status(decomposed_task_id, basic_info['task id'], 'Done') 
                response.success = True
                return response

            else:
                future_of_self_instruction = self.send_self_instruction(boss_id, self.robot_id, decomposed_task_id, task_content, self.my_function_description, Father_decomposed_from_task_id)
                if future_of_self_instruction.success:
                    self.update_task_status(decomposed_task_id, my_task_id, 'prepared')
                response.success = True
                return response

                # while True:
                #     future_of_self_instruction = self.send_self_instruction(boss_id, self.robot_id, my_task_id, task_content, self.my_function_description)
                #     if future_of_self_instruction.success:
                #         self.update_task_status(decomposed_task_id, my_task_id, 'prepared')
                #         break
            #####

            
        except Exception as e:
            self.get_logger().error(f'Error in execute_node_start_callback: {e}')
            response.success = False

        return response

    def update_task_status(self, decomposed_task_id, task_id, status):
        client = self.create_client(StringToBool, f'managed_task_info_update', callback_group=self.callback_group)
        req = StringToBool.Request()
        req.data = json.dumps({
            'Decomposed task id': decomposed_task_id,
            'Task id': task_id,
            'key': 'Task status',
            'keyvalue': status
        })
        self.get_logger().info(f'Updating task {task_id} status to "{status}" for decomposed task {decomposed_task_id} to boss {self.robot_id}')
        client.call_async(req)
        # 调用broadcaster_update服务
        self.update_broadcaster(task_id, 'Task status', status)




    def send_self_instruction (self, boss_id, robot_id, task_id, task_description, my_function_description, decomposed_task_id):
        client = self.create_client(StringToBool, f'/{boss_id}/cop_basic_info_add', callback_group=self.callback_group)
        req = StringToBool.Request()
        req.data = json.dumps({'robot id': robot_id, 'task id': task_id, 'task description': task_description,'my function description': my_function_description, 'decomposed task id': decomposed_task_id})
        self.get_logger().info(f'sending self instruction to {boss_id}')
        future = client.call(req)
        self.get_logger().info(f'respond from self instruction: {future}')
        return future


    def update_broadcaster(self, task_id, key, value):
        req = StringToBool.Request()
        req.data = json.dumps({
            'Task id': task_id,
            'key': key,
            'keyvalue': value
        })
        self.get_logger().info(f'Updating broadcaster with task {task_id} {key}: "{value}"')
        self.broadcaster_update_service_client.call_async(req)

    
    # def nav_2(self, robot_id, position):
    #     topic_name = f'/{robot_id}/goal_pose'
    #     publisher = self.create_publisher(PoseStamped, topic_name, 10)
    #     goal_pose = PoseStamped()
    #     goal_pose.header.frame_id = 'map'
    #     goal_pose.header.stamp = self.get_clock().now().to_msg()
    #     goal_pose.pose.position.x = position[0]  # 目标位置X坐标
    #     goal_pose.pose.position.y = position[1]  # 目标位置Y坐标
    #     distance = math.sqrt((goal_pose.pose.position.x - self.current_position[0])**2 + (goal_pose.pose.position.y - self.current_position[1])**2)
    #     while distance > 3:
    #         rclpy.spin_once(self, timeout_sec=0.1)  # 确保其他回调函数可以被调用
    #         distance = math.sqrt((goal_pose.pose.position.x - self.current_position[0])**2 + (goal_pose.pose.position.y - self.current_position[1])**2)
    #         publisher.publish(goal_pose)
    #         self.get_logger().info(f'Published goal pose to {topic_name}: {goal_pose}')
    #         time.sleep(1)
       
    # def call_cop_func(self, robot_id, function_name, args):
    #     client = self.create_client(StringToBool, f'{robot_id}/instruction_listener_service', callback_group=self.callback_group)
    #     req = StringToBool.Request()
    #     req.data = json.dumps({
    #         'function name': function_name,
    #         'function input': args
    #     })
    #     self.get_logger().info(f'Calling function {function_name} on {robot_id} with args {args}')
    #     client.call_async(req)

    # def lift_and_drop(self, robot_id, object_name, start_position, end_position):
    #     self.nav_2(robot_id, start_position)
    #     self.nav_2(robot_id, end_position)
        





def main(args=None):
    rclpy.init(args=args)
    node = ExecuteNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
