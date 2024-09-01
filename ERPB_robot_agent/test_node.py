import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float32MultiArray
from my_interfaces.srv import StringToBool
from execute_LLM_function import generate_function_trans_result
import time
import json

class ExeNode(Node):
    def __init__(self, namespace):
        super().__init__('execuate_node', namespace=namespace)
        self.callback_group = ReentrantCallbackGroup()
       

        self.Task_description = 'Transport the pump from [2,2] to [10,10]'
        self.My_robot_id = namespace
        self.collaborative_robots_basic_info =[{'robot id': 'robot2', 'task id': 'task2', 'task description': 'Move the pump from its location to the transport vehicle'}]
        self.environment_info = {
        'location of robot 1': [0, 0],
        'location of pump': [2, 2],
        "path_information": {
                        "start_point": "M",
                        "end_point": "B",  
                        "status": "passable"
                    },
            "drainage_pipe_location": "C",
            "ground_condition": "hard"
        }
        self.Dictionary_T_list='''{
        my_id: string; // My robot ID.
        transport_object: string; // Name of the object to be transported.
        transport_start_point: array; // Start point coordinates of the transport task, must be a letter point or specific coordinates. If unknown, enter False.
        transport_end_point: array; // End point coordinates of the transport task, must be a letter point or specific coordinates. If unknown, enter False.
        cop_1: string; // The robot ID of the collaborator responsible for moving the object to the transport vehicle. If unknown, enter False.
        object_position: string; // Index key of the object's location information in the environment information dictionary environment_info. Attention: fill in the index rather than the value. If unknown, enter False.
        my_self_position: string; // My position index key in the environment information dictionary environment_info. Attention: fill in the index rather than the value.  If unknown, enter False.
        }'''
        self.variable_list = ['object_position', 'my_self_position']
        self.T="""My task execution steps:
        Step 1: {my_id} moves to {transport_start_point}".
        Step 2: Let {cop_1} move to {transport_start_point}.
        Step 3: Let {cop_1} transport {transport_object} from {object_position} to {my_self_position}.
        Step 4: {my_id} moves to {transport_end_point}.

        Basic Functions of {my_id}:
        [
            {{
                "function_name": "nav_2(position)",
                "function_description": "Move the robot to a destination position.",
                "input_parameter_description": {{
                    "position": "The coordinates of the robot's destination, usually in the form of a 2D array like [x, y]."
                }}
            }},
            {{
                "function_name": "call_cop_func(cop_id, function, input)",
                "function_description": "Send a function call request to a collaborative robot with id cop_id, where the function name is function, and the input is input.",
                "input_parameter_description": {{
                    "cop_id": "The id of the collaborative robot receiving the function call request.",
                    "function": "The name of the requested function.",
                    "input": "An array like [a, b] consisting of the inputs required by the requested function."
                }}
            }}
        ]

        Basic Functions of {cop_1}:

        [
            {{
                "function_name": "lift_and_drop(obj, pos_a, pos_b)",
                "function_description": "Lift the object obj from its position a, move it to position b, and drop it.",
                "input_parameter_description": {{
                    "obj": "The object being lifted and moved.",
                    "pos_a": "The current position of the object.",
                    "pos_b": "The position to which the object needs to be moved."
                }}
            }}
        ]

        From the perspective of {my_id}, please write out the task execution steps from the above content in the form of functions. It is required to use only the functions of {my_id}, and you can use the call_cop_func function to call functions of other robots. Provide a list of function names with filled-in input content only, without defining any functions.
        Please note that if the input is in the form of dictionary indices, please maintain that format.
        """
        # Start Service Server
        self.execute_node_start_service= self.create_service(
            StringToBool,
            'execute_node_start_service',
            self.execute_node_start_callback,
            callback_group=self.callback_group
        )


        
        self.instructions = generate_function_trans_result(self.Task_description, self.My_robot_id, self.collaborative_robots_basic_info, self.environment_info, self.Dictionary_T_list, self.variable_list, self.T)
        self.nav_pub = self.create_publisher(Float32MultiArray, 'nav_topic', 10, callback_group=self.callback_group)
        self.cop_pub = self.create_publisher(String, 'cop_topic', 10, callback_group=self.callback_group)
        self.lift_pub = self.create_publisher(String, 'lift_topic', 10, callback_group=self.callback_group)

        self.execute_instructions()

        self.get_logger().info(f'ExeNode of {namespace} is ready.')

    
            
        
        

    def execute_instructions(self):
        for instruction in self.instructions:
            eval(instruction)
            time.sleep(10)

    def nav_2(self, position):
        # Publish navigation command
        msg = Float32MultiArray()
        msg.data = position
        self.nav_pub.publish(msg)
        self.get_logger().info(f'Navigating to position {position}')

    def call_cop_func(self, robot_name, func_name, params):
        # Publish cooperation function call
        msg = String()
        msg.data = f'Calling {func_name} on {robot_name} with params {params}'
        self.cop_pub.publish(msg)
        self.get_logger().info(msg.data)

    def lift_and_drop(self, item, location, destination):
        # Publish lift and drop command
        msg = String()
        msg.data = f'Lifting {item} from {location} to {destination}'
        self.lift_pub.publish(msg)
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    namespace = args[1]
    node = ExeNode(namespace)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
