import rclpy
from rclpy.node import Node
from my_interfaces.msg import Dictionaries

class DictionarySubscriber(Node):
    def __init__(self):
        super().__init__('dictionary_subscriber')
        self.subscription = self.create_subscription(
            Dictionaries,
            'dictionaries_topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # 解析 Dictionaries 消息
        dict_list = []
        for dict_msg in msg.dictionaries:
            dict_item = {}
            for key_value in dict_msg.entries:
                dict_item[key_value.key] = key_value.value
            dict_list.append(dict_item)
        
        # 打印解析后的字典列表
        self.get_logger().info(f'Received dictionaries: {dict_list}')
        # 这里可以进一步处理字典列表

def main(args=None):
    rclpy.init(args=args)
    node = DictionarySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
