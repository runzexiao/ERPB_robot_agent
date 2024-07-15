import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class PathInfoPublisher(Node):

    def __init__(self):
        super().__init__('environmetal_information_publisher_node')
        self.publisher_ = self.create_publisher(String, '/environmetal_information', 10)
        timer_period = 2  # 发布周期为2秒
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        data = {
            "path_information": {
                "start_point": "M",
                "end_point": "B",  
                "status": "passable"
            },
            "current_robot_location": "M",
            "water_pump_location": "C",
            "drainage_pipe_location": "C",
            "ground_condition": "hard"
        }
        msg = String()
        msg.data = json.dumps(data)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    path_info_publisher = PathInfoPublisher()
    rclpy.spin(path_info_publisher)
    path_info_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
