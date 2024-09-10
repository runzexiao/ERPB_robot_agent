import os
from langchain_community.chat_models import ChatOpenAI
from langchain.prompts import ChatPromptTemplate
from langchain.output_parsers import ResponseSchema
from langchain.output_parsers import StructuredOutputParser
from langchain.chains import LLMChain
from dotenv import load_dotenv, find_dotenv
import warnings
import random

warnings.filterwarnings('ignore')

# 加载环境变量
_ = load_dotenv(find_dotenv())

# 获取 API 密钥
api_key = os.getenv('OPENAI_API_KEY')

# 初始化 ChatOpenAI
llm = ChatOpenAI(temperature=0.0, openai_api_key=api_key, model_name='gpt-4o')

def extract_unique_abilities(bidding):
    abilities_set = {bid['Ability'] for bid in bidding}
    abilities_list = list(abilities_set)
    return abilities_list

def compare_bidding_with_task(task, robot_abilities):
    Judege_result_schema = ResponseSchema(name="Judge result", description="If you will hire one of them, fill in the ability the robot has in the original text; if you will not hire them, fill in the boolean value false here.")

    CAT_response_schemas = [Judege_result_schema]
    CAT_output_parser = StructuredOutputParser.from_response_schemas(CAT_response_schemas)
    CAT_format_instructions = CAT_output_parser.get_format_instructions()

    capacity_judge_method = """
  
    Now you are a robot recruitment manager, and you need to recruit a robot to be the core member and leader to complete the task '{task}'. A group of robots with the abilities listed in "{robot_abilities}" have come to apply. Who will you hire among them? Or will you hire none of them? Just tell me the result.
    Note that all the abilities mentioned here can be performed at any specific location.

    {CAT_format_instructions}
    """

    capacity_judge_prompt_template = ChatPromptTemplate.from_template(capacity_judge_method)

    chain_capacity_judge = LLMChain(llm=llm, prompt=capacity_judge_prompt_template, output_key="judge_result")

    result = chain_capacity_judge.run({
        "robot_abilities": robot_abilities,
        "task": task,
        "CAT_format_instructions": CAT_format_instructions
    })

    CAT_result_parser = CAT_output_parser.parse(result)

    return CAT_result_parser["Judge result"]

def compare_bidding_with_task_force(task, robot_abilities):
    Judege_result_schema = ResponseSchema(name="Judge result", description="Fill in the ability the hired robot has in the original text.")

    CAT_response_schemas = [Judege_result_schema]
    CAT_output_parser = StructuredOutputParser.from_response_schemas(CAT_response_schemas)
    CAT_format_instructions = CAT_output_parser.get_format_instructions()

    capacity_judge_method = """
  
    Now you are a robot recruitment manager, and you need to recruit a robot to be the core member and leader to complete the task '{task}'. A group of robots with the abilities listed in "{robot_abilities}" have come to apply. Which one will you hire among them? Just tell me the result.

    {CAT_format_instructions}
    """

    capacity_judge_prompt_template = ChatPromptTemplate.from_template(capacity_judge_method)

    chain_capacity_judge = LLMChain(llm=llm, prompt=capacity_judge_prompt_template, output_key="judge_result")

    print("Running compare_bidding_with_task_force for task:", task)
    result = chain_capacity_judge.run({
        "robot_abilities": robot_abilities,
        "task": task,
        "CAT_format_instructions": CAT_format_instructions
    })
    print("Received result from compare_bidding_with_task_force:", result)

    CAT_result_parser = CAT_output_parser.parse(result)

    return CAT_result_parser["Judge result"]

def evaluate_bids(task, bidding, compare_result):
    # Step 1: Filter bids by matching ability
    filtered_bids = [bid for bid in bidding if bid['Ability'] == compare_result]
    if not filtered_bids:
        return None  # No suitable bids found

    # Step 2: Sort filtered bids by efficiency (descending)
    sorted_bids = sorted(filtered_bids, key=lambda x: -x['Efficiency'])
    
    # Step 3: Identify the best bid(s)
    best_bids = [sorted_bids[0]]
    
    for bid in sorted_bids[1:]:
        if bid['Efficiency'] == best_bids[0]['Efficiency']:
            best_bids.append(bid)
        else:
            break
    
    # Step 4: Randomly select one bid if there are multiple best bids
    selected_bid = random.choice(best_bids)
    
    return selected_bid['Robot id']

def select_robot_for_task(task, bidding):
    bidding_abilities = extract_unique_abilities(bidding)
    compare_result = compare_bidding_with_task(task, bidding_abilities)
    if compare_result:
        selected_robot_id = evaluate_bids(task, bidding, compare_result)
        return selected_robot_id
    else:
        return False

# Example usage in another script
if __name__ == "__main__":
    task = 'Build a drainage pipe driven by a water pump from point A to point B, with the water pump installed at point B.'
    bidding1 = [
        {'Robot id': 'robot_2', 'Ability': 'Lifting objects such as piles and pumps', 'Efficiency': 5},
        {'Robot id': 'robot_3', 'Ability': 'Build a drainage pipe', 'Efficiency': 5},
        {'Robot id': 'robot_1', 'Ability': 'Lifting objects such as piles and pumps', 'Efficiency': 5},
        {'Robot id': 'robot_4', 'Ability': 'Build a drainage pipe', 'Efficiency': 5}
    ]

    selected_robot_id = select_robot_for_task(task, bidding1)
    print(f"Selected Robot ID: {selected_robot_id}")
