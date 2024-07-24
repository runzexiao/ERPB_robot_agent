import json
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from my_interfaces.srv import StringToBool
from std_msgs.msg import String

class BiddingTestNode(Node):
    def __init__(self, namespace, task_id, robot_id, ability, efficiency):
        super().__init__('bidding_evaluation_test', namespace=namespace)
        self.namespace = namespace
        self.task_id = task_id
        self.robot_id = robot_id
        self.ability = ability
        self.efficiency= efficiency
        self.callback_group = ReentrantCallbackGroup()

        # 等待bidding_evaluation_update_service服务
        self.bidding_update_client = self.create_client(
            StringToBool,
            f'/robot1/task_{self.task_id}/bidding_evaluation_update_service',
            callback_group=self.callback_group
        )

        self.wait_for_services([self.bidding_update_client])
        # 定时器，用于周期性地发送投标请求
        self.timer = self.create_timer(5.0, self.send_bidding_request)

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

    def send_bidding_request(self):
        self.timer.cancel()
        bidding_data = {
            'Task id': self.task_id,
            'Robot id': self.robot_id,
            'Ability': self.ability,
            'Efficiency': self.efficiency
        }

        req = StringToBool.Request()
        req.data = json.dumps(bidding_data)

        future = self.bidding_update_client.call_async(req)
        future.add_done_callback(self.handle_bidding_response)

    def handle_bidding_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'{self.robot_id} successfully sent a bid for task {self.task_id}')
            else:
                self.get_logger().error(f'{self.robot_id} failed to send a bid for task {self.task_id}')
        except Exception as e:
            self.get_logger().error(f'Error sending bid: {str(e)}')

def main(args=None):
    rclpy.init(args=args)

    # 示例：多个机器人为一个任务投标
    task_ids = ['2', '3']
    robot_ids = ['robot2', 'robot3']
    abilities = ['Transport the pump', 'build water pipe']
    efficiencies = [1,1]
    
    nodes = []
    executor = MultiThreadedExecutor()

    for i in range(0, len(robot_ids)):
        namespace = robot_ids[i]
        robot_id = robot_ids[i]
        task_id = task_ids[i]
        ability = abilities[i]
        efficiency = efficiencies[i]
          # 示例命名空间，可以参数化
        node = BiddingTestNode(namespace, task_id, robot_id , ability, efficiency)
        nodes.append(node)
        executor.add_node(node)

    try:
        executor.spin()
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
