import rclpy
from rclpy.node import Node
from my_interfaces.msg import Dictionaries, Dictionary, KeyValue
from my_interfaces.srv import StringToBool, DictToBool, TaskDependencies, BoolStringToBool
from std_srvs.srv import Trigger, SetBool

class BiddingNodeTester(Node):
    def __init__(self):
        super().__init__('bidding_test')

        self.first_start_flag = True
        self.publish_num = 0
        # Publisher
        self.publisher = self.create_publisher(Dictionaries, 'broadcaster_published', 10)
        
        # Create the service servers that BiddingNode requests
        self.update_service = self.create_service(DictToBool, 'broadcaster_update_service', self.update_service_callback)
        self.dependencies_service = self.create_service(TaskDependencies, 'get_task_dependencies', self.dependencies_service_callback)
        self.decomposition_service = self.create_service(DictToBool, 'robot1/task_decomposition_service', self.decomposition_service_callback)

        self.bidding_evaluation_services = {}

        # Service clients
        self.start_client = self.create_client(StringToBool, 'robot1/bidding_node_start_service')
        self.result_client = self.create_client(BoolStringToBool, 'robot1/bidding_result_input_service')

        self.timer = self.create_timer(1.0, self.publish_task_list)
        # Call the start service
        
        self.get_logger().info('BiddingNodeTester is ready.')

    def publish_task_list(self):
        msg = Dictionaries()
        task1 = Dictionary()
        task1.entries.append(KeyValue(key='Task id', value='1'))
        task1.entries.append(KeyValue(key='Priority', value='1'))
        task1.entries.append(KeyValue(key='Content', value='knick the box'))
        task1.entries.append(KeyValue(key='Boss id', value='Robot_1'))
        task1.entries.append(KeyValue(key='Task type', value='type1'))

        task2 = Dictionary()
        task2.entries.append(KeyValue(key='Task id', value='2'))
        task2.entries.append(KeyValue(key='Priority', value='0'))
        task2.entries.append(KeyValue(key='Content', value='Install the water pump at pont B.'))
        task2.entries.append(KeyValue(key='Boss id', value='Robot_2'))
        task2.entries.append(KeyValue(key='Task type', value='type1'))
        
        msg.dictionaries.append(task1)
        msg.dictionaries.append(task2)
        
        self.publisher.publish(msg)
        # self.get_logger().info(f'Published task list {self.publish_num} to /broadcaster_published')
        self.publish_num += 1

        self.call_start_service()

        # Dynamically create bidding evaluation service
        task_id = '2'
        boss_robot_id = 'Robot_2'
        service_name = f'{boss_robot_id}/task_{task_id}/bidding_evaluation_update_service'
        if service_name not in self.bidding_evaluation_services:
            self.bidding_evaluation_services[service_name] = self.create_service(DictToBool, service_name, self.bidding_evaluation_callback)
            self.get_logger().info(f'Created service: {service_name}')

    def bidding_evaluation_callback(self, request, response):
        self.get_logger().info(f'Received bidding evaluation request: {request.data}')
        response.success = True
        return response

    def update_service_callback(self, request, response):
        self.get_logger().info('Received request to update blackboard.')
        response.success = True
        return response

    def dependencies_service_callback(self, request, response):
        print(1234)
        self.get_logger().info('Received request for task dependencies.')
        response.reachable_tasks = ['1']  # Example of reachable tasks
        return response

    def decomposition_service_callback(self, request, response):
        self.get_logger().info('Received request to decompose task.')
        response.success = True
        return response

    def call_start_service(self):
        start_available = self.start_client.wait_for_service(timeout_sec=1.0)
        if start_available and self.first_start_flag:
            # request = StringToBool.Request()
            # request.data = 'start'
            # future = self.start_client.call_async(request)
            
        
            # Set a timer to call the result service after 10 seconds
            self.create_timer(10.0, self.call_result_service_timer)
            self.first_start_flag = False
        elif not start_available:
            self.get_logger().info('Waiting for bidding_node_start_service to become available...')

    def call_result_service_timer(self):
        self.call_result_service(True, 'task_2')

    def call_result_service(self, result, task_id):
        if not self.result_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for bidding_result_input_service to become available...')
            return

        request = BoolStringToBool.Request()
        request.bool_data = result
        request.string_data = task_id
        future = self.result_client.call_async(request)


        # if future.result() is not None:
        #     self.get_logger().info('Bidding result input service call succeeded')
        # else:
        #     self.get_logger().info('Bidding result input service call failed')

def main(args=None):
    rclpy.init(args=args)
    tester_node = BiddingNodeTester()
    
    rclpy.spin(tester_node)
    tester_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
