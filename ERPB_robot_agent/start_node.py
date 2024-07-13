import rclpy
from rclpy.node import Node
from my_interfaces.msg import Dictionaries
from my_interfaces.srv import StringToBool

class StartNode(Node):
    def __init__(self, namespace):
        super().__init__('start_node', namespace=namespace)

        self.black_board_sub = self.create_subscription(
            Dictionaries,
            '/black_board_published',
            self.black_board_callback,
            10
        )
        self.operator_called = False
        self.start_bidding = False
        self.bidding_node_start_service_client = self.create_client(StringToBool, 'bidding_node_start_service')
        self.operator_task_processing_start_service_client = self.create_client(StringToBool, 'operator_task_processing_start_service')
        self.operator_task_service = self.create_service(StringToBool, 'operator_task_input_service', self.operator_task_callback)
        
        self.wait_for_services()

        self.get_logger().info(f'Start Node of {namespace} is ready.')
    
    def wait_for_services(self):
        while True:
            bidding_node_ready = self.bidding_node_start_service_client.wait_for_service(timeout_sec=1.0)
            operator_task_ready = self.operator_task_processing_start_service_client.wait_for_service(timeout_sec=1.0)

            if bidding_node_ready and operator_task_ready: 
                self.get_logger().info('Both services are now available.')
                break  

            if not bidding_node_ready:
                self.get_logger().info('bidding_node_start_service not available, waiting again...')
            if not operator_task_ready:
                self.get_logger().info('operator_task_processing_start_service not available, waiting again...')
    
    def operator_task_callback(self, request, response):
        if self.start_bidding:  
            self.get_logger().info(f'Bidding node has already started.')
            response.success = False  
        else:
            self.get_logger().info(f'Received a task from operator: {request.data}')
            self.call_service('operator_task_processing_start_service', request.data)
            self.operator_called = True
            response.success = True
        return response
    
    def black_board_callback(self, msg):
        if msg.dictionaries:  # If the message is not empty
            if not self.start_bidding:
                self.get_logger().info('Received non-empty black_board message.')
                self.call_service('bidding_node_start_service', "start")
                self.start_bidding = True
        else:
            self.get_logger().info('Waiting for non-empty black_board_published message.')
    
    def call_service(self, service_name, request_data):
        if service_name == 'bidding_node_start_service':
            req = StringToBool.Request()
            req.data = request_data
            self.bidding_node_start_service_client.call_async(req)
            self.get_logger().info(f'Calling service {service_name} with data {request_data}')
        elif service_name == 'operator_task_processing_start_service': 
            req = StringToBool.Request()
            req.data = request_data
            self.operator_task_processing_start_service_client.call_async(req)
            self.get_logger().info(f'Calling service {service_name} with data {request_data}')
        else:
            self.get_logger().error(f'service {service_name} not found.')

def main(args=None):
    rclpy.init(args=args)
    namespace = 'robot1'  # Example namespace, can be parameterized
    node = StartNode(namespace)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
