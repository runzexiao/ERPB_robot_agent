import sys
import os

# 添加当前目录到搜索路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from my_interfaces.msg import Dictionaries, Dictionary, KeyValue
from my_interfaces.srv import StringToBool,DictToBool,TaskDependencies,BoolStringToBool
from std_srvs.srv import Trigger,SetBool
from std_msgs.msg import String
from bidding_LLM_function import compare_ability_with_task_fast
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import copy
import yaml
import os
import json




class BiddingNode(Node):
    def __init__(self):
        super().__init__('bidding_node')
         # 使用 ReentrantCallbackGroup 允许并发回调
        self.callback_group = ReentrantCallbackGroup()
        ##Parameters
        #package name
        self.pkg_name = "ERPB_robot_agent"

        # 机器人自身的id
        self.my_robot_id = self.get_namespace().lstrip('/')
  
        # 机器人自身id的文件名
        #params file name
        self.params_file_name = self.my_robot_id + "_params.yaml"
        
        # 总体任务列表
        self.task_list =[]
      
        # 已投标但被拒绝的任务id列表
        self.rejected_task_ids = set()
        # CAT_result_parser判别为false的任务id列表
        self.unscannable_task_ids = set()
        #能力列表
        self.ability_list=[]
        # 能力效率字典
        self.abilities_efficiency_dict = {}
        # 加载机器人自身能力参数
        self.load_abilities()
        self.bidding_result={"bidding_result": True, "task_id": "9999"}
        self.lock = threading.Lock()
        
        # Defination of Subscribers
        self.broadcaster_sub = self.create_subscription(
            String,
            '/broadcaster_published',
            self.broadcaster_callback,
            10,
            callback_group=self.callback_group
        )

        # # 机器人自身位置
        # self.pose =[]
        # # 机器人群的位置信息
        # self.robot_pose_dict = {}
        # self.environment_robots_pose_sub = self.create_subscription(
        #     DictionaryPose2D,
        #     'environment_robots_pose',
        #     self.environment_robots_pose_callback,
        #     10,
        #     callback_group=self.callback_group
        # )

        # Defination of Service_server
        self.bidding_node_start_service = self.create_service(
            StringToBool, 
            'bidding_node_start_service', 
            self.bidding_node_start_callback,
            callback_group=self.callback_group
        )
        self.bidding_result_input_service = self.create_service(
            BoolStringToBool, 
            'bidding_result_input_service', 
            self.bidding_result_input_callback,
            callback_group=self.callback_group
        )

        # Defination of Service_client
        self.broadcaster_update_service_client = self.create_client(
            StringToBool, 
            '/broadcaster_task_info_update',
            callback_group=self.callback_group
        )
        self.get_task_dependencies_client = self.create_client(
            TaskDependencies, 
            '/get_task_dependencies',
            callback_group=self.callback_group
        )
        self.task_decomposition_service_client = self.create_client(
            StringToBool, 
            'task_decomposition_service',
            callback_group=self.callback_group
        )

       
    
        # Wait for services are already defined
        self.wait_for_services([self.broadcaster_update_service_client, self.get_task_dependencies_client, self.task_decomposition_service_client])
        self.get_logger().info(f'Bidding Node of {self.my_robot_id} is ready.')

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
     
       
        self.ability_list_underscored = [item.replace(' ', '_') for item in self.ability_list]

        # Iterate through each ability and get the efficiency parameter
        for ability in self.ability_list_underscored:
            efficiency = config[ability]['efficiency']
            self.abilities_efficiency_dict[ability] = efficiency
        
        self.get_logger().info(f'Ability efficiency dict: {self.abilities_efficiency_dict}')
    
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

    def dictionary_request_fill_in(self,dict):
        dict_msg = Dictionary()
        for key, value in dict.items():
            key_value_msg = KeyValue()
            key_value_msg.key = key
            key_value_msg.value = str(value)
            dict_msg.entries.append(key_value_msg)
           
        return dict_msg

    
    def wait_for_bidding_result(self, task_id):
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)  # 确保其他回调函数可以被调用
            self.get_logger().info(f"{self.bidding_result['task_id']}")
            self.get_logger().info(f"{task_id}")
            if self.bidding_result["task_id"] == task_id:
                break
        return self.bidding_result["bidding_result"]


    def bid_for_task(self, task_id, ability, boss_robot_id):
        self.get_logger().info("Bid for task before service call")
        ability_slash= ability.replace(" ", "_")
        bid_packet = {
            'Robot id': self.my_robot_id,
            'Ability': ability,
            'Efficiency': self.abilities_efficiency_dict[ability_slash],
            'Task id': task_id
        }
        # Create a service client for bidding_evaluation_service
        self.bidding_evaluation_update_service_client = self.create_client(StringToBool, f'/{boss_robot_id}/task_{task_id}/bidding_evaluation_update_service', callback_group=self.callback_group)
        self.wait_for_services([self.bidding_evaluation_update_service_client])
        req = StringToBool.Request()
        req.data = json.dumps(bid_packet)
        self.get_logger().info(f"Bidding for task {task_id} with bid packet: {bid_packet} to {boss_robot_id}...")
        self.bidding_evaluation_update_service_client.call_async(req)
        bidding_result = self.wait_for_bidding_result(task_id)
        return bidding_result
        

 
        
                
    def bidding_node_start_callback(self, request, response):
        self.bidding_t = request.data
        self.get_logger().info(self.bidding_t)
        while True:
            rclpy.spin_once(self, timeout_sec=0.4)  # 确保其他回调函数可以被调用
            self.get_logger().info('Bidding started')
            if self.bidding_t == "start":
                self.get_logger().info('get start branch')
                task_list_now = copy.deepcopy(self.task_list)
                # if self.task_list == []:
                #     self.get_logger().info('task_list is empty')
               
            else:
                self.td_request = TaskDependencies.Request()
                self.td_request.task_id = self.bidding_t
                task_dependencies=self.get_task_dependencies_client.call(self.td_request)

                task_list_now_all = copy.deepcopy(self.task_list)
                task_list_now = [d for d in task_list_now_all if d["Task id"] in task_dependencies.reachable_tasks]
            
            task_list_now.sort(key=lambda x: x['Priority'], reverse=True)
            self.get_logger().info(f'Sorted list for bidding {task_list_now}')
            for task in task_list_now:
                task_id = task['Task id']
                task_content = task['Content']
                boss_robot_id = task['Boss id']
                task_type = task['Task type']
                priority = task['Priority']
                Father_decomposed_from_task_id = task['Decomposed from task id']
                Shadow_boss = task['Shadow boss']
                # 判断任务是否已经被自己投标过

                self.get_logger().info('Before compare')
                # 跳过已经投标但被拒绝的任务和CAT_result_parser判别为false的任务
                if task_id in self.rejected_task_ids or task_id in self.unscannable_task_ids:
                    continue

                # 判断任务内容是否有自己能够完成的部分
                #chosen_result, CAT_result_parser = compare_ability_with_task(task_content, self_abilities)
                self.get_logger().info(f'self.ability_list:{self.ability_list}')
                CAT_result_parser = compare_ability_with_task_fast(task_content, self.ability_list)
                #print(chosen_result, CAT_result_parser)
                self.get_logger().info(f'Get ability to deal with the task: {CAT_result_parser}')

                if CAT_result_parser:
                    # 投标
                    #success = bid_for_task(task_id, chosen_result, boss_robot_id)
                    success = self.bid_for_task(task_id, CAT_result_parser, boss_robot_id)
                    if success:
                        # 假设投标成功后，结束任务处理循环
                        self.get_logger().info(f"Successfully bid for task {task_id}")
                        #请求开启self.task_decomposition_service_client，并且把有关列表清空
                        decompose_packet = {
                                'My id': self.my_robot_id,
                                'Task content': task_content,
                                'Priority': priority,
                                'My ability': CAT_result_parser,
                                'From start flag': True,
                                'Decomposed task type':task_type,
                                'Decomposed task id':task_id,
                                'Father decomposed from task id':Father_decomposed_from_task_id,
                                'Boss id': boss_robot_id,
                                'Shadow boss' : Shadow_boss
                            }
                        self.get_logger().info(f"Decompose packet: {decompose_packet}")
                        reqd = StringToBool.Request()
                        # reqd.data = self.dictionary_request_fill_in(decompose_packet)
                        reqd.data =json.dumps(decompose_packet)
                        self.task_decomposition_service_client.call_async(reqd)

                

                        broadcaster_updated_packet1 = {
                                'Task id': task_id,
                                'key': 'Task status',
                                'keyvalue': 'Claimed'
                            }
                        
                        reqb1 = StringToBool.Request()
                        reqb1.data = json.dumps(broadcaster_updated_packet1)
                        self.broadcaster_update_service_client.call_async(reqb1)

                        broadcaster_updated_packet2 = {
                                'Task id': task_id,
                                'key': 'Worker id',
                                'keyvalue': self.my_robot_id
                            }
                        
                        reqb2 = StringToBool.Request()
                        reqb2.data = json.dumps(broadcaster_updated_packet2)
                        self.broadcaster_update_service_client.call_async(reqb2)


                        break
                    else:
                        # 评标失败，将任务id加入被拒绝的任务列表
                        self.rejected_task_ids.add(task_id)
                else:
                    # CAT_result_parser判别为false，将任务id加入unscannable_task_ids
                    self.unscannable_task_ids.add(task_id)
            else:
                # 如果没有任务成功，则重新读取任务列表
                self.get_logger().info("Re-reading task list...")
                
                continue  # 重新开始while循环，不执行下面的break

            break  # 如果for循环中使用了break，这里才会执行，终止while循环
        response.success = True
            


    def bidding_result_input_callback(self, request, response):
        self.bidding_result = {"bidding_result": request.bool_data, "task_id": request.string_data}
        # print(self.bidding_result)
        response.success = True
        return response
    
    
    def broadcaster_callback(self, msg):
        # 使用临时变量构建新的任务列表
        new_task_list = json.loads(msg.data)
        
        
        # 完成构建后再赋值给 self.task_list
        self.task_list = new_task_list
        # self.get_logger().info(f'Received published task list: {self.task_list}')
        
        # 打印解析后的字典列表
        # self.get_logger().info(f'Received task list: {self.task_list}')
        # 这里可以进一步处理字典列表


    # def environment_robots_pose_callback(self, msg):
    #     for key_value in msg.entries:
    #         self.robot_pose_dict[key_value.key] = key_value.pose2d
    #     self.pose=[self.robot_pose_dict[self.my_robot_id].x, self.robot_pose_dict[self.my_robot_id].y,self.robot_pose_dict[self.my_robot_id].theta]

    

def main(args=None):
    rclpy.init(args=args)
    node = BiddingNode()
    # 使用 MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
