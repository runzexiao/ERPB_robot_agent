import json
import networkx as nx
import matplotlib.pyplot as plt
from same_test_LLM_function import compare_two_text_content
import random

class TaskManager:

    def __init__(self):
        self.global_task_list = []  # 全局任务列表
        self.global_published_task_list = []  # 全局发布中的任务列表
        self.global_dependency_graph = nx.DiGraph()  # 全局任务依赖关系图
        self.current_task_id = 4  # 初始化任务ID生成器
        self.temp_to_global_id_map = {}

    def generate_new_task_id(self):
        new_id = str(self.current_task_id)
        self.current_task_id += 1
        return new_id

    def process_data_packet(self, data_packet):
        decomposed_task_id = data_packet.get('Decomposed from task id')
        

        local_task_list = data_packet['Task list']  # 获取数据包中的任务列表
        local_dependency_graph = data_packet['Local dependency graph']  # 获取数据包中的局部依赖关系

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
        return [{'Task id': task['Task id'], 'Priority': task['Priority'], 'Content': task['Content'], 'Boss id': task['Boss id'], 'Task status': task['Task status']} for task in sorted_list]

    def get_published_task_list(self):
        return [{'Task id': task['Task id'], 'Priority': task['Priority'], 'Content': task['Content'], 'Boss id': task['Boss id'], 'Task status': task['Task status']} for task in self.global_published_task_list]

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


# 测试代码
if __name__ == '__main__':
    # 创建TaskManager实例
    task_manager = TaskManager()

    # 初始化全局任务列表和全局依赖关系图
    initial_tasks = [
        {'Task id': '1', 'Priority': 3, 'Content': 'Clean up the site.', 'Boss id': 'robot_99', 'Task status': 'Published'},
        {'Task id': '2', 'Priority': 2, 'Content': 'Build a drainage pipe driven by a water pump from point A to point B, with the water pump installed at point B.', 'Boss id': 'robot_1', 'Task status': 'Published'},
        {'Task id': '3', 'Priority': 1, 'Content': 'Excavate the drainage ditch.', 'Boss id': 'robot_77', 'Task status': 'Published'}
    ]
    
    task_manager.global_task_list.extend(initial_tasks)
    task_manager.global_dependency_graph.add_edges_from([
        ('3', '2'),
        ('2', '1')
    ])

    # 定义两个数据包
    data_packet1 = {
        "Task list": [
            {
                "Task type": "My task",
                "Content": "Transport the water pump from point C to the point B.",
                "Priority": 3,
                "Temp id": "1",
                "Boss id": "robot_2"
            },
            {
                "Task type": "preliminary task",
                "Content": "Lift the water pump at point C onto the transport vehicle.",
                "Priority": 4,
                "Temp id": "2",
                "Boss id": "robot_2"
            }
        ],
        "Local dependency graph": {
            "1": ["2"],
            "2": []
        },
        "Decomposed from task id": "5"
    }

    data_packet2 = {
        'Task list': [
            {'Task type': 'My task', 'Content': 'Install the water pump at point B.', 'Priority': 2, 'Temp id': '1', 'Boss id': 'robot_1'},
            {'Task type': 'preliminary task', 'Content': 'Transport the water pump from position C to position B.', 'Priority': 3, 'Temp id': '2', 'Boss id': 'robot_1'},
            {'Task type': 'independent task', 'Content': 'Build a drainage pipe from point A to point B.', 'Priority': 2, 'Temp id': '3', 'Boss id': 'robot_1'}
        ],
        'Local dependency graph': {'1': ['2'], '2': [], '3': []},
        "Decomposed from task id": "2"
    }


    # 处理数据包
    #task_manager.process_data_packet(data_packet1)
    task_manager.process_data_packet(data_packet2)
    task_manager.process_data_packet(data_packet1)

    # 打印合并后的任务列表
    print("Merged and Sorted Task List:")
    for task in task_manager.get_sorted_task_list():
        print(task)

    # 打印发布中的任务列表
    print("\nPublished Task List:")
    for task in task_manager.get_published_task_list():
        print(task)

    # 打印合并后的依赖关系图
    print("\nMerged Dependency Graph:")
    print(json.dumps(task_manager.get_dependency_graph(), indent=2))

    # 绘制依赖关系图
    task_manager.draw_dependency_graph()