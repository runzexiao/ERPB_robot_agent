import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class PathInfoSubscriber(Node):

    def __init__(self):
        super().__init__('environmental_information_subscriber_node')
        self.subscription = self.create_subscription(
            String,
            '/environmetal_information',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用的变量警告

    def listener_callback(self, msg):
        self.get_logger().info('Received message: "%s"' % msg.data)
        data = json.loads(msg.data)
        self.process_data(data)

    def process_data(self, data):
        # 在这里处理接收到的数据
        path_info = data["path_information"]
        current_location = data["current_robot_location"]
        water_pump_location = data["water_pump_location"]
        drainage_pipe_location = data["drainage_pipe_location"]
        ground_condition = data["ground_condition"]

        if path_info["status"]:
            print(current_location[0]+current_location[1])


            # 打印解析后的数据
            self.get_logger().info('Path Information: start_point=%s, end_point=%s, status=%s' % 
                                (path_info["start_point"], path_info["end_point"], path_info["status"]))
            self.get_logger().info('Current Robot Location: %s' % current_location)
            self.get_logger().info('Water Pump Location: %s' % water_pump_location)
            self.get_logger().info('Drainage Pipe Location: %s' % drainage_pipe_location)
            self.get_logger().info('Ground Condition: %s' % ground_condition)

def main(args=None):
    rclpy.init(args=args)
    path_info_subscriber = PathInfoSubscriber()
    rclpy.spin(path_info_subscriber)
    path_info_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
