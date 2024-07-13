import rclpy
from rclpy.node import Node
from my_interfaces.msg import Dictionaries, Dictionary, KeyValue

class DictionaryPublisher(Node):
    def __init__(self):
        super().__init__('dictionary_publisher')
        self.publisher_ = self.create_publisher(Dictionaries, 'dictionaries_topic', 10)
        timer_period = 1.0  # 发布周期（秒）
        self.timer = self.create_timer(timer_period, self.publish_dictionaries)

    def publish_dictionaries(self):
        # 假设你有一个字典列表
        dict_list = [
            {"key1": "value1", "key2": "value2"},
            {"keyA": "valueA", "keyB": "valueB"}
        ]

        # 创建 Dictionaries 消息
        dictionaries_msg = Dictionaries()

        for dict_item in dict_list:
            dict_msg = Dictionary()
            for key, value in dict_item.items():
                key_value_msg = KeyValue()
                key_value_msg.key = key
                key_value_msg.value = value
                dict_msg.entries.append(key_value_msg)
            dictionaries_msg.dictionaries.append(dict_msg)

        # 发布消息
        self.publisher_.publish(dictionaries_msg)
        self.get_logger().info('Publishing dictionaries')

def main(args=None):
    rclpy.init(args=args)
    node = DictionaryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
