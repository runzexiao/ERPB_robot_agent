import os
from langchain_community.chat_models import ChatOpenAI
from langchain.prompts import ChatPromptTemplate
from langchain.output_parsers import ResponseSchema
from langchain.output_parsers import StructuredOutputParser
from langchain.chains import LLMChain
from dotenv import load_dotenv, find_dotenv
import warnings

warnings.filterwarnings('ignore')

# 加载环境变量
_ = load_dotenv(find_dotenv())

# 获取 API 密钥
api_key = os.getenv('OPENAI_API_KEY')

# 初始化 ChatOpenAI
llm = ChatOpenAI(temperature=0.0, openai_api_key=api_key, model_name='gpt-4o')

def generate_function_trans_result(Task_description, My_robot_id, collaborative_robots_basic_info, environment_info, Dictionary_T_list, variable_list, T):
    input_list_schema = ResponseSchema(name="dictionary T",
                                 description="Fill in the completed dictionary T.")

    EFI_response_schemas = [input_list_schema]
    EFI_output_parser = StructuredOutputParser.from_response_schemas(EFI_response_schemas)
    EFI_format_instructions = EFI_output_parser.get_format_instructions()

    input_fill_in_method = '''
    Task Description: {Task_description}
    My robot ID: {My_robot_id}
    Basic information dictionary of collaborative robots:
    {collaborative_robots_basic_info}
    Environment information dictionary:
    environment_info = {environment_info}
    Required input dictionary T:
    {Dictionary_T_list}
    Fill in the dictionary T for me based on the information above. 
    {EFI_format_instructions}
    '''

    input_fill_in_template = ChatPromptTemplate.from_template(input_fill_in_method)

    chain_input_fill_in = LLMChain(llm=llm, prompt=input_fill_in_template, output_key="DT")

    EFI_result = chain_input_fill_in.run({
      "Task_description": Task_description,
      "My_robot_id": My_robot_id,
      "collaborative_robots_basic_info": collaborative_robots_basic_info,
      "environment_info": environment_info,
      "Dictionary_T_list": Dictionary_T_list,
      "EFI_format_instructions": EFI_format_instructions})
    
    input_list = EFI_output_parser.parse(EFI_result)['dictionary T']

    input_list["object_position"] = f"environment_info['{input_list['object_position']}']"
    input_list["my_self_position"] =f"environment_info['{input_list['my_self_position']}']"

    A = input_list

    function_trans_schema = ResponseSchema(name="function list",
                                 description="Fill in the python list of function names with filled-in input.")

    FTR_response_schemas = [function_trans_schema]
    FTR_output_parser = StructuredOutputParser.from_response_schemas(FTR_response_schemas)
    FTR_format_instructions = FTR_output_parser.get_format_instructions()

    method_for_now = T.format(**A)

    function_trans_method ="""
    {method_for_now}
    {FTR_format_instructions}
    """

    function_trans_template = ChatPromptTemplate.from_template(function_trans_method)

    chain_function_trans = LLMChain(llm=llm, prompt=function_trans_template, output_key="FL")

    FTR_result = chain_function_trans.run({
      'method_for_now': method_for_now,
      'FTR_format_instructions': FTR_format_instructions
      })
    
    function_trans_result = FTR_output_parser.parse(FTR_result)['function list']
    
    function_trans_result = [instr.replace('environment_info', 'self.environment_info') for instr in function_trans_result]
    function_trans_result = ['self.' + instr for instr in function_trans_result]

    return function_trans_result

if __name__ == "__main__":
    Task_description = 'Transport the pump from [2,2] to [10,10]'
    My_robot_id = 'robot1'
    collaborative_robots_basic_info =[{'robot id': 'robot2', 'task id': 'task2', 'task description': 'Move the pump from its location to the transport vehicle'}]
    environment_info = {
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
    Dictionary_T_list='''{
      my_id: string; // My robot ID.
      transport_object: string; // Name of the object to be transported.
      transport_start_point: array; // Start point coordinates of the transport task, must be a letter point or specific coordinates. If unknown, enter False.
      transport_end_point: array; // End point coordinates of the transport task, must be a letter point or specific coordinates. If unknown, enter False.
      cop_1: string; // The robot ID of the collaborator responsible for moving the object to the transport vehicle. If unknown, enter False.
      object_position: string; // Index key of the object's location information in the environment information dictionary environment_info. Attention: fill in the index rather than the value. If unknown, enter False.
      my_self_position: string; // My position index key in the environment information dictionary environment_info. Attention: fill in the index rather than the value.  If unknown, enter False.
    }'''
    variable_list = ['object_position', 'my_self_position']
    T="""My task execution steps:
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

    function_trans_result = generate_function_trans_result(Task_description, My_robot_id, collaborative_robots_basic_info, environment_info, Dictionary_T_list, variable_list, T)
    print(function_trans_result)
