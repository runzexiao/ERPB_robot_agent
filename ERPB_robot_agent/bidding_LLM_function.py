# task_ability_comparator.py

import os
from langchain_community.chat_models import ChatOpenAI
from langchain.prompts import ChatPromptTemplate
from langchain.output_parsers import ResponseSchema
from langchain.output_parsers import StructuredOutputParser
from langchain.chains import LLMChain
from langchain.chains import SequentialChain
from dotenv import load_dotenv, find_dotenv
import warnings

warnings.filterwarnings('ignore')

# 加载环境变量
_ = load_dotenv(find_dotenv())

# 获取 API 密钥
api_key = os.getenv('OPENAI_API_KEY')

# 初始化 ChatOpenAI
llm = ChatOpenAI(temperature=0.0, openai_api_key=api_key, model_name='gpt-4o')

def compare_ability_with_task(task_content, self_abilities):
    ########### Input #########################
    capacity_choesn_method = """
    Please select the most essential key item from my capability list {self_abilities} for completing the literal description part of task {task_content}. Please answer me with the original text of the item in the list in string. Only tell me the result.
    """

    capacity_choesn_prompt_template = ChatPromptTemplate.from_template(capacity_choesn_method)

    chain_capacity_choesn = LLMChain(llm=llm, prompt=capacity_choesn_prompt_template, output_key="chosen_result")

    Judege_result_schema = ResponseSchema(name="Judge result", description="If you will hire it, fill in the boolean value true here; if you will not hire it, fill in the boolean value false here.")

    CAT_response_schemas = [Judege_result_schema]
    CAT_output_parser = StructuredOutputParser.from_response_schemas(CAT_response_schemas)
    CAT_format_instructions = CAT_output_parser.get_format_instructions()

    capacity_judge_method = """
    Now you are a robot recruitment manager, and you need to recruit a robot to be the core member to complete the task '{task_content}'. A robot with the ability "{chosen_result}" is available. Will you hire it? Just tell me the result.
    {CAT_format_instructions}
    """

    capacity_judge_prompt_template = ChatPromptTemplate.from_template(capacity_judge_method)

    chain_capacity_judge = LLMChain(llm=llm, prompt=capacity_judge_prompt_template, output_key="judge_result")

    overall_chain = SequentialChain(
        chains=[chain_capacity_choesn, chain_capacity_judge],
        input_variables=["self_abilities", "task_content", "CAT_format_instructions"],
        output_variables=["chosen_result", "judge_result"],
        verbose=False
    )

    overall_result = overall_chain({
        "self_abilities": self_abilities,
        "task_content": task_content,
        "CAT_format_instructions": CAT_format_instructions
    })

    CAT_result_parser = CAT_output_parser.parse(overall_result["judge_result"])
    
    return overall_result["chosen_result"], CAT_result_parser["Judge result"]

def compare_ability_with_task_fast(task_content, self_abilities):
    ########### Input #########################
   
    Judege_result_schema = ResponseSchema(name="Judge result", description="If you will hire one of them, fill in the ability the robot has; if you will not hire them, fill in the boolean value false here.")

    CAT_response_schemas = [Judege_result_schema]
    CAT_output_parser = StructuredOutputParser.from_response_schemas(CAT_response_schemas)
    CAT_format_instructions = CAT_output_parser.get_format_instructions()

    capacity_judge_method = """
  
    Now you are a robot recruitment manager, and you need to recruit a robot to be the core member and leader to complete the task '{task_content}'. A group of robots with the abilities listed in "{self_abilities}" have come to apply. Who will you hire among them? Or will you hire none of them? Just tell me the result.

    {CAT_format_instructions}
    """

    capacity_judge_prompt_template = ChatPromptTemplate.from_template(capacity_judge_method)

    chain_capacity_judge = LLMChain(llm=llm, prompt=capacity_judge_prompt_template, output_key="judge_result")

    result= chain_capacity_judge.run({
        "self_abilities": self_abilities,
        "task_content": task_content,
        "CAT_format_instructions": CAT_format_instructions
    })

    CAT_result_parser = CAT_output_parser.parse(result)

    return CAT_result_parser["Judge result"]
    
