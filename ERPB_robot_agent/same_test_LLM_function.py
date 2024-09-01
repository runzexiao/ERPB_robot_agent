import os
from langchain_community.chat_models import ChatOpenAI
from langchain.prompts import ChatPromptTemplate
from langchain.output_parsers import ResponseSchema
from langchain.output_parsers import StructuredOutputParser
from langchain.chains import LLMChain
from dotenv import load_dotenv, find_dotenv
import warnings

from fuzzywuzzy import fuzz




warnings.filterwarnings('ignore')

# 加载环境变量
_ = load_dotenv(find_dotenv())

# 获取 API 密钥
api_key = os.getenv('OPENAI_API_KEY')

# 初始化 ChatOpenAI
llm = ChatOpenAI(temperature=0.0, openai_api_key=api_key, model_name='gpt-4o')





def compare_two_text_content(A, B):
    
    # 计算两个句子的相似度
    similarity = fuzz.ratio(A, B)
    
    if similarity > 70:
        print(f"{A} and {B} are using GPT, with sim = {similarity}")
        comparison_result_schema = ResponseSchema(name="comparison result",
                                    description="Fill in the boolean value of the comparison result.")
                        

        response_schemas = [comparison_result_schema]
        output_parser = StructuredOutputParser.from_response_schemas(response_schemas)
        format_instructions = output_parser.get_format_instructions()

        comparison_method = '''
        Instruction: Check the task content of task A enclosed in square brackets and the task B enclosed in angle brackets. 

        If the meaning of the task A is the same as task B, then output the comparison result as the boolean value true.
        otherwise, output the comparison result as the boolean value false.
        Only output the comparison result, do not output any other information.

        [task A: {A}]
        <task B: {B}>
        {format_instructions}
        '''

        comparison_template = ChatPromptTemplate.from_template(comparison_method)

        chain_comparison = LLMChain(llm=llm, prompt=comparison_template, output_key="RES")

        result = chain_comparison.run({
        "A": A,
        "B": B,
        "format_instructions": format_instructions
        })

        comparison_result = output_parser.parse(result)['comparison result']

        if comparison_result:
          print(f"{A} and {B} are the same")

        return comparison_result
    else:
        #print(f"{A} and {B} are not using GPT, with sim = {similarity}")
        return False


# A = "Task unload the water pump from truck C to the point B."
# B = "Drop down the water pump from truck C to the point B."
# print(compare_two_text_content(A, B))



