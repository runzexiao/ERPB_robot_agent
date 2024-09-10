import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from my_interfaces.srv import StringToBool
import json

class EnvInfoPublisher(Node):

    def __init__(self):
        super().__init__('environmental_information_publisher_node')
        self.publisher_ = self.create_publisher(String, '/environmental_information', 10)
        timer_period = 2  # 发布周期为2秒
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 初始化机器人的位置信息字典
        self.robot_positions = {'c30r_0':[0, 0], 'c30r_1':[0, 0], 'c30r_2':[0, 0], 'c30r_3':[0, 0], 'c30r_4':[0, 0], 'c30r_5':[0, 0]}
        # 初始化环境信息
        self.env_info = {
            "pipe_information": {
                "pipe_status": "folded",
                "pipe_start_location": [0, 0],
                "pipe_end_location": [0, 0]
            },
            "robot_location": self.robot_positions,
            "water_pump_location": [0, 0],
            "ground_condition": "hard",
            "paths in the environment": "Any two points in the environment is passable."
        }

        # 订阅每个机器人的位置话题
        self.subscribers = []
        for i in range(0, 6):  # 6个机器人
            topic_name = f'/c30r_{i}/base_link/pose'
            subscriber = self.create_subscription(
                PoseStamped,
                topic_name,
                self.create_listener_callback(f'c30r_{i}'),
                10)
            self.subscribers.append(subscriber)
        
        # 创建一个服务，用于更新环境信息中的参数
        self.srv = self.create_service(StringToBool, '/update_environment_info', self.update_environment_info_callback)

    def create_listener_callback(self, robot_name):
        def listener_callback(msg):
            # 提取x和y坐标
            x = float(round(msg.pose.position.x))
            y = float(round(msg.pose.position.y))
            self.robot_positions[robot_name] = [x, y]
            self.env_info['robot_location'] = self.robot_positions

            # self.get_logger().info(f'{robot_name} position: x={x}, y={y}')
        return listener_callback

    def timer_callback(self):
        # 发布环境信息
        msg = String()
        msg.data = json.dumps(self.env_info)
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
    
    def update_environment_info_callback(self, request, response):
        try:
            request_dict = json.loads(request.data)
            key = request_dict['key']
            keyvalue = request_dict['keyvalue']
            
            if key in self.env_info:
                self.env_info[key] = keyvalue
                self.get_logger().info(f'Environment info updated with {key}: {keyvalue}')
                response.success = True
            elif key in self.env_info['pipe_information']:
                self.env_info['pipe_information'][key] = keyvalue
                self.get_logger().info(f'Pipe information updated with {key}: {keyvalue}')
                response.success = True
            else:
                self.get_logger().error(f'Key {key} not found in environment information.')
                response.success = False

        except Exception as e:
            self.get_logger().error(f'Error in update_environment_info_callback: {e}')
            response.success = False

        return response

def main(args=None):
    rclpy.init(args=args)
    path_info_publisher = EnvInfoPublisher()
    rclpy.spin(path_info_publisher)
    path_info_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
