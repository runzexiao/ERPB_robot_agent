import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from my_interfaces.srv import StringToDict, StringToBool, StringToString
from my_interfaces.srv import TaskDependencies  # 假设你有这个服务类型
from my_interfaces.msg import Dictionary, KeyValue,Dictionaries
from same_test_LLM_function import compare_two_text_content
import json
import networkx as nx
import matplotlib.pyplot as plt
import random

class TaskBroadcasterNode(Node):
    def __init__(self):
        super().__init__('task_broadcaster_node')
        self.callback_group = ReentrantCallbackGroup()
        self.global_task_list = []  # 全局任务列表
        self.global_published_task_list = []  # 全局发布中的任务列表
        self.global_dependency_graph = nx.DiGraph()  # 全局任务依赖关系图
        self.current_task_id = 1  # 初始化任务ID生成器
        self.temp_to_global_id_map = {}
        self.add_flag = False

        # 发布者
        self.task_publisher = self.create_publisher(String, '/broadcaster_published', 10)
        self.timer = self.create_timer(2.0, self.publish_published_tasks)

        # 服务
        self.add_service = self.create_service(
            StringToDict,
            '/broadcaster_add_service',
            self.broadcaster_add_service_callback,
            callback_group=self.callback_group
        )
        self.update_service = self.create_service(
            StringToBool,
            '/broadcaster_task_info_update',
            self.broadcaster_task_info_update_callback,
            callback_group=self.callback_group
        )
        self.get_dependencies_service = self.create_service(
            TaskDependencies,
            '/get_task_dependencies',
            self.get_task_dependencies_callback,
            callback_group=self.callback_group
        )
        self.broadcaster_query_service = self.create_service(
            StringToString,
            '/broadcaster_query_service',
            self.broadcaster_query_service_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Task Broadcaster Node is ready.')
    
    def generate_new_task_id(self):
        new_id = str(self.current_task_id)
        self.current_task_id += 1
        return new_id
    # 检查图是否为空
    def is_graph_empty(self, graph):
        return graph.number_of_nodes() == 0 and graph.number_of_edges() == 0

    def process_data_packet(self, data_packet):
        decomposed_task_id = data_packet.get('Decomposed from task id')
        

        local_task_list = data_packet['Task list']  # 获取数据包中的任务列表
        local_dependency_graph = data_packet['Local dependency graph']  # 获取数据包中的局部依赖关系

        print(f"Processing data packet for decomposed task id: {decomposed_task_id}")
        print(f"Local task list: {local_task_list}")
        print(f"Local dependency graph: {local_dependency_graph}")

        # 处理任务并分配新的任务ID
        self.temp_to_global_id_map = {}  # 映射局部临时ID到全局新ID
        for new_task in local_task_list:
            # 查找是否已有相同内容的任务
            existing_task = next((task for task in self.global_task_list if task['Task id'] != decomposed_task_id and compare_two_text_content(task['Content'], new_task['Content'])), None)
            if existing_task:
                task_id = existing_task['Task id']
                priority_diff = abs(existing_task['Priority'] - new_task['Priority'])
                if new_task['Priority'] < existing_task['Priority']:
                    self.update_priority(new_task['Temp id'], priority_diff, local_dependency_graph)
                else:
                    self.update_priority(existing_task['Task id'], priority_diff)
                existing_task['Priority'] = max(existing_task['Priority'], new_task['Priority'])
                if existing_task['Task status'] != 'Published':
                    shadow_data = {"Shadow boss id": new_task['Boss id'], "Shadow decomposed id": new_task['Decomposed from task id'], "Shadow task id": task_id}
                    e_worker_id = existing_task['Worker id']
                    

                    client = self.create_client(StringToBool, f'{e_worker_id}/manager_shadow_boss_append', callback_group=self.callback_group)
                    req = StringToBool.Request()
                    req.data = json.dumps({
                            'Decomposed task id': task_id,
                            'Shadow boss': shadow_data
                            })
                    self.get_logger().info(f'Append shadow boss {shadow_data} to worker {e_worker_id}')
                    client.call_async(req)
                else:
                    shadow_data = {"Shadow boss id": new_task['Boss id'], "Shadow decomposed id": new_task['Decomposed from task id'], "Shadow task id": task_id}
                    existing_task['Shadow boss'].append(shadow_data)
                
            else:
                task_id = self.generate_new_task_id()  # 分配新的任务ID
                new_task['Task id'] = task_id
                # 设置任务执行状态
                new_task['Task status'] = 'Claimed' if new_task['Task type'] == 'My task' else 'Published'
                self.global_task_list.append(new_task)  # 添加新任务到全局任务列表
            self.temp_to_global_id_map[new_task['Temp id']] = task_id

        # 更新局部依赖关系图中的任务ID
        updated_local_dependency_graph = {}
        for node, edges in local_dependency_graph.items():
            global_node_id = self.temp_to_global_id_map[node]
            updated_edges = [self.temp_to_global_id_map[edge] for edge in edges]
            updated_local_dependency_graph[global_node_id] = updated_edges

        # 合并依赖关系图
        local_graph = nx.DiGraph(updated_local_dependency_graph)  # 将更新后的局部依赖关系图转换为networkx的有向图
        for node in local_graph.nodes:
            if node not in self.global_dependency_graph:
                self.global_dependency_graph.add_node(node)  # 添加新节点到全局依赖关系图

        for edge in local_graph.edges:
            if edge not in self.global_dependency_graph.edges:
                self.global_dependency_graph.add_edge(*edge)  # 添加新边到全局依赖关系图

        if decomposed_task_id:
            # 更新全局任务列表中对应任务的状态
            for task in self.global_task_list:
                if task['Task id'] == decomposed_task_id:
                    task['Task status'] = 'Decomposed'
                    break

            # 从全局依赖关系图中删除该任务的节点，并更新依赖关系
            if decomposed_task_id in self.global_dependency_graph:
                successors = list(self.global_dependency_graph.predecessors(decomposed_task_id))
                predecessors = list(self.global_dependency_graph.successors(decomposed_task_id))
                self.global_dependency_graph.remove_node(decomposed_task_id)
    

                # 获取新的任务并添加到全局依赖关系图中
                for new_task in data_packet['Task list']:
                    task_type = new_task['Task type']
                    if task_type in ['My task', 'independent task', 'collaborative task', 'incremental task']:
                        task_id = self.temp_to_global_id_map[new_task['Temp id']]
                        

                        # 继承原有的依赖关系
                        for predecessor in predecessors:
                            self.global_dependency_graph.add_edge(task_id, predecessor)
                        for successor in successors:
                            self.global_dependency_graph.add_edge(successor, task_id)
                        print(self.global_dependency_graph.nodes)

        # 更新全局发布中的任务列表
        self.global_published_task_list = [task for task in self.global_task_list if task['Task status'] == 'Published']
        return self.temp_to_global_id_map

    def update_priority(self, task_id, priority_diff, dependency_graph=None):
        graph = self.global_dependency_graph if dependency_graph is None else nx.DiGraph(dependency_graph)  # 选择要更新的图
        tasks_to_update = set()  # 用于存储需要更新的任务ID
        self.collect_tasks_to_update(task_id, graph, tasks_to_update)  # 收集所有需要更新的任务ID

        for task_id in tasks_to_update:
            task = next(task for task in self.global_task_list if task['Task id'] == task_id)  # 找到对应的任务
            task['Priority'] += abs(priority_diff)  # 更新任务的优先级

    def collect_tasks_to_update(self, task_id, graph, tasks_to_update):
        for dependent_task in nx.descendants(graph, task_id):
            tasks_to_update.add(dependent_task)  # 将所有依赖于task_id的任务ID添加到更新列表中

    def get_sorted_task_list(self):
        sorted_list = sorted(self.global_task_list, key=lambda x: x['Priority'], reverse=True)  # 按优先级排序全局任务列表
        return [{'Task id': task['Task id'], 'Priority': task['Priority'], 'Content': task['Content'], 'Task type': task['Task type'], 'Boss id': task['Boss id'], 'Task status': task['Task status'], 'Decomposed from task id': task['Decomposed from task id'], 'Shadow boss': task['Shadow boss']} for task in sorted_list]

    def get_published_task_list(self):
        return [{'Task id': task['Task id'], 'Priority': task['Priority'], 'Content': task['Content'], 'Task type': task['Task type'], 'Boss id': task['Boss id'], 'Task status': task['Task status'], 'Decomposed from task id': task['Decomposed from task id'], 'Shadow boss': task['Shadow boss']} for task in self.global_published_task_list]

    def get_dependency_graph(self):
        return nx.node_link_data(self.global_dependency_graph)  # 获取全局依赖关系图的数据

    def draw_dependency_graph(self):
        plt.figure(figsize=(12, 8))
        
        # 获取连通分量
        subgraphs = list(nx.weakly_connected_components(self.global_dependency_graph))

        # 获取每个节点的优先级
        priority_map = {task['Task id']: task['Priority'] for task in self.global_task_list}
        
        # 计算每个连通分量在画面中的位置
        pos = {}
        
        # 设置每个节点的层次位置
        max_priority = max(priority_map.values())
        priority_levels = {i: [] for i in range(max_priority + 1)}
        for node, priority in priority_map.items():
            priority_levels[priority].append(node)

        # 计算每层的高度位置，按照优先级将画面垂直方向平均划分
        num_levels = max_priority + 1
        layer_height = 1.0 / num_levels

        for level, nodes in priority_levels.items():
            num_nodes = len(nodes)
            if num_nodes > 0:
                x_positions = [(i + 1) / (num_nodes + 1) for i in range(num_nodes)]
                for i, node in enumerate(nodes):
                    y_offset = (level + 0.5) * layer_height
                    pos[node] = (x_positions[i], y_offset)

        nx.draw(self.global_dependency_graph, pos, with_labels=True, node_size=3000, node_color="lightblue", font_size=10, font_weight="bold", edge_color="gray")
        edge_labels = nx.get_edge_attributes(self.global_dependency_graph, 'weight')
        nx.draw_networkx_edge_labels(self.global_dependency_graph, pos, edge_labels=edge_labels, font_color="red")
        plt.title("Task Dependency Graph", fontsize=15)
        plt.show()  # 显示图形

    def dictionary_request_fill_in(self,dict):
        dict_msg = Dictionary()
        for key, value in dict.items():
            key_value_msg = KeyValue()
            key_value_msg.key = key
            key_value_msg.value = str(value)
            dict_msg.entries.append(key_value_msg)
           
        return dict_msg
    def broadcaster_add_service_callback(self, request, response):
        try:
            data_packet = json.loads(request.data)
            temp_to_global_map = self.process_data_packet(data_packet)
            dict_msg = self.dictionary_request_fill_in(temp_to_global_map)
            response.dict = dict_msg
            self.get_logger().info(f'Task is added')
            self.get_logger().info("Merged and Sorted Task List:")
            # print(1234455)
            for task in self.get_sorted_task_list():
                self.get_logger().info(json.dumps(task))

            # 打印发布中的任务列表
            self.get_logger().info("\nPublished Task List:")
            for task in self.get_published_task_list():
                print(task)

            # 打印合并后的依赖关系图
            self.get_logger().info("\nMerged Dependency Graph:")
            self.get_logger().info(json.dumps(self.get_dependency_graph(), indent=2))
            self.add_flag = True
        except Exception as e:
            self.get_logger().error(f'Error in broadcaster_add_service_callback: {e}')
            response.dict = Dictionary()
        return response

    def broadcaster_task_info_update_callback(self, request, response):
        try:
            request_dict = json.loads(request.data)
            task_id = request_dict['Task id']
            key = request_dict['key']
            keyvalue = request_dict['keyvalue']

            for task in self.global_task_list:
                if task['Task id'] == task_id:
                    task[key] = keyvalue
                    self.get_logger().info(f'Task {task_id} updated with {key}: {keyvalue}')
                    response.success = True
                    return response

            self.get_logger().error(f'Task {task_id} not found in global task list')
            response.success = False
        except Exception as e:
            self.get_logger().error(f'Error in managed_task_info_update_callback: {e}')
            response.success = False

        return response

    def get_task_dependencies_callback(self, request, response):
        try:
            # 使用 networkx 的 descendants 方法获取所有从 task_id 出发单向可达的节点
            task_id = request.task_id
            if task_id in self.global_dependency_graph:
                descendants = nx.descendants(self.global_dependency_graph, task_id)
                response.reachable_tasks = list(descendants)
                self.get_logger().info(f'Task {task_id} dependencies: {response.reachable_tasks}')
            else:
                response.reachable_tasks = []
                self.get_logger().warn(f'Task {task_id} not found in the global dependency graph.')
        except Exception as e:
            self.get_logger().error(f'Error in get_task_dependencies_callback: {e}')
            response.reachable_tasks = []

        return response

    def broadcaster_query_service_callback(self, request, response):
        try:
            Task_id = request.request_data
            # find the task content of Task_id in global_task_list
            for task in self.global_task_list:
                if task['Task id'] == Task_id:
                    response.response_data = task['Content']
                    return response
            response.response_data = "Task not found"
            return response
        except Exception as e:
            self.get_logger().error(f'Error in broadcaster_query_service_callback: {e}')

    def publish_published_tasks(self):
        try:
            # 获取当前的已发布任务列表
            published_task_list = self.get_published_task_list()

            # 将任务列表转换为JSON格式的字符串
            published_task_json = json.dumps(published_task_list)

            # 创建并发布String消息
            msg = String()
            msg.data = published_task_json
            self.task_publisher.publish(msg)
            if not self.is_graph_empty(self.global_dependency_graph) and self.add_flag:
            # 绘制依赖关系图
                self.draw_dependency_graph()
                self.add_flag = False
            # self.get_logger().info('Published current task list in JSON format.')

        except Exception as e:
            self.get_logger().error(f'Error in publish_published_tasks: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TaskBroadcasterNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
