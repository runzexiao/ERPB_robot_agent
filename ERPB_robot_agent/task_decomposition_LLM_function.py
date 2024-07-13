import os
import warnings
import time
import sys
import json
from langchain_community.chat_models import ChatOpenAI
from langchain.prompts import ChatPromptTemplate
from langchain.output_parsers import ResponseSchema
from langchain.output_parsers import StructuredOutputParser
from langchain.chains import LLMChain
from dotenv import load_dotenv, find_dotenv

warnings.filterwarnings('ignore')

# Load environment variables
_ = load_dotenv(find_dotenv())

# Get API key
api_key = os.getenv('OPENAI_API_KEY')

# Initialize ChatOpenAI
llm = ChatOpenAI(temperature=0.0, openai_api_key=api_key, model_name='gpt-4o')


def create_data_packet(task_list, robot_id):
    temp_id_map = {}
    local_dependency_graph = {}
    current_temp_id = 1

    # Assign temporary IDs
    for task in task_list:
        temp_id = str(current_temp_id)
        current_temp_id += 1
        task['Temp id'] = temp_id
        task['Robot id'] = robot_id
        temp_id_map[task['Content']] = temp_id

    # Generate dependency graph based on task type
    for task in task_list:
        temp_id = task['Temp id']
        task_type = task['Task type']
        dependencies = []

        if task_type == 'My task':
            dependencies = [temp_id_map[t['Content']] for t in task_list if t['Task type'] == 'preliminary task']
        elif task_type == 'subsequent task':
            dependencies = [temp_id_map[t['Content']] for t in task_list if t['Task type'] == 'My task']

        local_dependency_graph[temp_id] = dependencies

    data_packet = {
        'Task list': task_list,
        'Local dependency graph': local_dependency_graph
    }

    return data_packet


def create_task_string(task_list):
    task_list = [f"Task {task.rstrip('.')}" for task in task_list]
    task_string = " and ".join(task_list)
    return task_string


def remove_duplicate_dicts(input_list):
    seen = set()
    result = []
    for d in input_list:
        frozenset_items = frozenset(d.items())
        if frozenset_items not in seen:
            result.append(d)
            seen.add(frozenset_items)
    return result


def decompose_task(
    overall_task_dict, 
    my_capability, 
    environmental_information, 
    knowledge_input_list, 
    knowledge_of_decomposition,
    knowledge_template,
    robot_id,
    start_from_scratch=True
):
    Decomposed_Task_list = []
    stop_flag = 0

    Overall_Task = overall_task_dict["Content"]

    if start_from_scratch:
        print("Starting task decomposition from scratch...")

        # Task division
        print("Dividing tasks...")
        D_My_task_schema = ResponseSchema(name="My task",
                                        description="The part of the overall task that my capability can accomplish, described as precisely and specifically as possible.")
        D_Remaining_task_schema = ResponseSchema(name="Remaining task",
                                                description="tasks that my capability cannot complete.If there is no remaining task,fill in the boolean value false.")

        D_response_schemas = [D_My_task_schema, D_Remaining_task_schema]
        D_output_parser = StructuredOutputParser.from_response_schemas(D_response_schemas)
        D_format_instructions = D_output_parser.get_format_instructions()

        task_division_method = """
        Instruction: Check the overall task information enclosed in double quotes and my capability information enclosed in single quotes. Complete the following condition evaluation and task publishing. Only tell me the published results.

        if(my capability is sufficient to complete the overall task)
        then [publish my task: The specific part of the overall task that my capability can accomplish]
        else [publish my task: The specific part of the overall task that my capability can accomplish,
        publish remaining tasks: specific tasks that my capability cannot complete]

        "{Overall_Task}"
        '{My_capability}'
        {D_format_instructions}
        """

        task_division_prompt_template = ChatPromptTemplate.from_template(task_division_method)

        chain_division_bi = LLMChain(llm=llm, prompt=task_division_prompt_template, 
                            output_key="task_division_result")

        D_result = chain_division_bi.run({
            "Overall_Task": Overall_Task,
            "My_capability": my_capability,
            "D_format_instructions": D_format_instructions
        })

        D_result_parser = D_output_parser.parse(D_result)
        print("Task division result:", D_result_parser)

        Decomposed_Task_list.append({"Task type": "My task", "Content": D_result_parser['My task'],"Priority": overall_task_dict["Priority"]})

        print("Task division complete. My task and remaining tasks identified.")

        # Extract knowledge input
        print("Extracting knowledge input...")
        if knowledge_input_list != "nothing":
            Knowledge_input_schema = ResponseSchema(name="Dictionary T",
                                        description="Fill in the completed dictionary T.")
            Information_resuest_task_schema = ResponseSchema(name="Information request task",
                                                description="If an information request task is required, fill in the task content. If all items in Dictionary T are known, fill in Boolean value false.")

            EKI_response_schemas = [Knowledge_input_schema, Information_resuest_task_schema]
            EKI_output_parser = StructuredOutputParser.from_response_schemas(EKI_response_schemas)
            EKI_format_instructions = EKI_output_parser.get_format_instructions()

            Knowledge_input_extract_method = """Instruction: Check the task content within double quotes and the environmental information within single quotes. Check each item in the JSON dictionary T:{Knowledge_input_list} to determine whether each item is known.

            if(all items in T are known)
            then [fill in the contents of each key-value pair as required and output the filled JSON dictionary T]
            else [identify the unknown items and issue an information request task for the missing information.]]

            "Task content: {Task_content}"
            'Environmental information: {Environmental_information}'

            Only output the content in the following Json format without additional information. Output format:
            {EKI_format_instructions}
            """

            Knowledge_input_extract_prompt_template = ChatPromptTemplate.from_template(Knowledge_input_extract_method)

            chain_knowledge_input_extract = LLMChain(llm=llm, prompt=Knowledge_input_extract_prompt_template)

            EKI_result = chain_knowledge_input_extract.run({
                "Knowledge_input_list": knowledge_input_list,
                "Task_content": Overall_Task,
                "Environmental_information": environmental_information,
                "EKI_format_instructions": EKI_format_instructions
            })

            EKI_result_parser = EKI_output_parser.parse(EKI_result)
            print("Knowledge input extraction result:", EKI_result_parser)

            if EKI_result_parser["Information request task"] == False:
                Dictionary_T = EKI_result_parser["Dictionary T"]
            else:
                Decomposed_Task_list.append({"Task type": "Information request task", "Content": EKI_result_parser['Information request task'],"Priority": overall_task_dict["Priority"] + 1})
                stop_flag = 1
        else:
            Dictionary_T = {}

        print("Knowledge input extraction complete.")

        while stop_flag == 1:
            print("Waiting for required information to be obtained...")
            time.sleep(1)
            sys.exit(0)

        # Knowledge based task publisher
        print("Publishing tasks based on knowledge...")

        if Dictionary_T:
            knowledge = knowledge_template.format(**Dictionary_T)
        else:
            knowledge = knowledge_template

        knowledge_task_pub_schema = ResponseSchema(name="Published task list based on knowledge",
                                    description="Fill in here with a list of all the JSON dictionaries of tasks to be published. If here is no task should to be published, Fill in here an empty list.")

        KTP_response_schemas = [knowledge_task_pub_schema]
        KTP_output_parser = StructuredOutputParser.from_response_schemas(KTP_response_schemas)
        KTP_format_instructions = KTP_output_parser.get_format_instructions()

        knowledge_check_method = """
        Instruction: Check the environment information enclosed in double quotes. Based on the following conditions, publish the corresponding tasks. Note that only the tasks determined by the condition evaluation should be published. If the conditions are insufficient, please return that the conditions are insufficient. Only tell me the published results.

        "{Environmental_information}"

        {knowledge}

        {knowledge_of_decomposition}

        {KTP_format_instructions}
        """

        knowledge_check_prompt_template = ChatPromptTemplate.from_template(knowledge_check_method)

        chain_knowledge_task_pub = LLMChain(llm=llm, prompt=knowledge_check_prompt_template)

        KTP_result = chain_knowledge_task_pub.run({
            "Environmental_information": environmental_information, 
            "knowledge": knowledge, 
            "knowledge_of_decomposition": knowledge_of_decomposition, 
            "KTP_format_instructions": KTP_format_instructions
        })

        KTP_result_parser = KTP_output_parser.parse(KTP_result)
        print("Knowledge based task publishing result:", KTP_result_parser)

        if KTP_result_parser["Published task list based on knowledge"]:
            Decomposed_Task_list.extend(KTP_result_parser["Published task list based on knowledge"])

        print("Knowledge based task publishing complete.")
    else:
        print("Starting task decomposition from Unresolved Remaining Task Handling Module...")
        D_result_parser = {'Remaining task': Overall_Task}
        KTP_result_parser = {"Published task list based on knowledge": []}

    # Unresolved Remaining Task Handling
    print("Handling unresolved remaining tasks...")
    Remaining_task = D_result_parser['Remaining task']

    if Remaining_task != False:
        PIT_schema = ResponseSchema(name="Published independent tasks",
                                    description="Fill in here with a list of all the JSON dictionaries of tasks to be published. If there is nothing to publish, fill in an empty list.")

        URTH_response_schemas = [PIT_schema]
        URTH_output_parser = StructuredOutputParser.from_response_schemas(URTH_response_schemas)
        URTH_format_instructions = URTH_output_parser.get_format_instructions()

        independent_task_finder_method = """
        Instruction: Check the overall task information enclosed in double quotes and the already published task information enclosed in single quotes. Based on the following conditions, publish the corresponding tasks. If the conditions are insufficient, please return that the conditions are insufficient. Only tell me the published results.

        "Overall task: {Remaining_task}"
        'Already published task list: {Already_published_task_list}'

        if (the already published task completely covers the overall task)
        then [there is no need to publish any task here.]
        else [publish independent tasks: the tasks of the overall task that the already published task did not solve.]

        {knowledge_of_decomposition}
        {URTH_format_instructions}
        """

        independent_task_finder_prompt_template = ChatPromptTemplate.from_template(independent_task_finder_method)

        chain_independent_task_finder = LLMChain(llm=llm, prompt=independent_task_finder_prompt_template)

        URTH_result = chain_independent_task_finder.run({
            "Remaining_task": Remaining_task,
            "Already_published_task_list": Decomposed_Task_list,
            "knowledge_of_decomposition": knowledge_of_decomposition,
            "URTH_format_instructions": URTH_format_instructions,
        })

        URTH_result_parser = URTH_output_parser.parse(URTH_result)
        print("Unresolved remaining task handling result:", URTH_result_parser)

        if URTH_result_parser["Published independent tasks"]:
            Decomposed_Task_list.extend(URTH_result_parser["Published independent tasks"])

        print("Unresolved remaining task handling complete.")

    # Final checker Module
    print("Running final checker module...")
    supplementary_task_finder_counter = 0  # Initialize counter
    supplementary_task_finder_counter_threshold = 3
    Supplementary_task_checker_success = True
    Suggestion_Task_list = []

    while supplementary_task_finder_counter < supplementary_task_finder_counter_threshold:
        Decomposed_Task_list_content = [d['Content'] for d in Decomposed_Task_list]
        Decomposed_Task_list_content_string = create_task_string(Decomposed_Task_list_content)

        supplementary_task_finder_method = """
            Instruction: Review the overall task enclosed in double quotes and the already published tasks enclosed in single quotes. Based on the already published tasks, think about what is still missing to complete the overall task. Provide the most important missing task. Only give me the final result, no need for the reasoning process.
            "Overall task:{Overall_Task}"
            'Already published tasks:{Decomposed_Task_list_content_string}'
            """

        supplementary_task_finder_prompt_template = ChatPromptTemplate.from_template(supplementary_task_finder_method)

        chain_supplementary_task_finder = LLMChain(llm=llm, prompt=supplementary_task_finder_prompt_template, output_key="Supplementary_Task")

        STF_result = chain_supplementary_task_finder.run({
                "Overall_Task": Overall_Task,
                "Decomposed_Task_list_content_string": Decomposed_Task_list_content_string
        })
        Supplementary_Task = STF_result
        print("Supplementary task finder result:", Supplementary_Task)

        Testing_task_checker_schema = ResponseSchema(name="Testing task checker result",
                                        description="If issue is raised, fill in boolean value true; if not, fill in boolean value false.")

        TTC_response_schemas = [Testing_task_checker_schema]
        TTC_output_parser = StructuredOutputParser.from_response_schemas(TTC_response_schemas)
        TTC_format_instructions = TTC_output_parser.get_format_instructions()

        testing_task_checker_method = """
                Instruction: Check the supplementary tasks enclosed in angle brackets, perform the following conditional judgment.
                if(supplementary tasks belong to testing, secure, inspection or acceptance tasks) 
                then [Raise an issue]
                else if (supplementary task is "ensure something" or "secure something")
                then [Raise an issue]

                <Supplementary Tasks: {Supplementary_Task}>
                {TTC_format_instructions}
                """

        testing_task_checker_template = ChatPromptTemplate.from_template(testing_task_checker_method)

        chain_testing_task_checker = LLMChain(llm=llm, prompt=testing_task_checker_template, output_key="Testing_task_checker_result")

        TTC_result = chain_testing_task_checker.run({
                "Supplementary_Task": Supplementary_Task,
                "TTC_format_instructions": TTC_format_instructions
        })
        Is_Test = TTC_output_parser.parse(TTC_result)['Testing task checker result']
        print("Testing task checker result:", Is_Test)

        Necessity_checker_schema = ResponseSchema(name="Necessity checker result",
                                        description="If supplementary tasks are necessary, fill in boolean value true; if not, fill in boolean value false.")

        NC_response_schemas = [Necessity_checker_schema]
        NC_output_parser = StructuredOutputParser.from_response_schemas(NC_response_schemas)
        NC_format_instructions = NC_output_parser.get_format_instructions()

        Necessity_checker_method = """
                Instruction: Check the supplementary tasks enclosed in angle brackets and overall task enclosed in double quotes, perform the following conditional judgment.
                if (the overall task can not be completed without the supplementary tasks)
                then [determine supplementary tasks are necessary]

                <Supplementary Tasks: {Supplementary_Task}>
                "Overall task:{Overall_Task}"
                
                {NC_format_instructions}
                """

        Necessity_checker_template = ChatPromptTemplate.from_template(Necessity_checker_method)

        chain_Necessity_checker = LLMChain(llm=llm, prompt=Necessity_checker_template, output_key="Necessity_checker_result")

        NC_result = chain_Necessity_checker.run({
                "Supplementary_Task": Supplementary_Task,
                "Overall_Task": Overall_Task,
                "NC_format_instructions": NC_format_instructions
        })
        Is_Unnecessary = NC_output_parser.parse(NC_result)['Necessity checker result']
        print("Necessity checker result:", Is_Unnecessary)

        Redundancy_checker_schema = ResponseSchema(name="Redundancy checker result",
                                        description="If Supplementary Task is too late to issue, fill in boolean value true; if not, fill in boolean value false.")

        RC_response_schemas = [Redundancy_checker_schema]
        RC_output_parser = StructuredOutputParser.from_response_schemas(RC_response_schemas)
        RC_format_instructions = RC_output_parser.get_format_instructions()

        Redundancy_checker_method = """
        In the case that {Decomposed_Task_list_content_string} have been completed, is it too late to issue Supplementary Task '{Supplementary_Task}' to complete task '{Overall_Task}'?  \n

        Note: In all task descriptions, the literal meaning is the entirety of the task. Do not assume that other tasks are implicitly completed once these tasks are finished. \n

        {RC_format_instructions}

        Only give me final output.
        """

        Redundancy_checker_template = ChatPromptTemplate.from_template(Redundancy_checker_method)

        chain_Redundancy_checker = LLMChain(llm=llm, prompt=Redundancy_checker_template, output_key="Redundancy_checker_result")

        RC_result = chain_Redundancy_checker.run({
                "Supplementary_Task": Supplementary_Task,
                "Overall_Task": Overall_Task,
                "Decomposed_Task_list_content_string": Decomposed_Task_list_content_string,
                "RC_format_instructions": RC_format_instructions
        })
        Is_Redundant = RC_output_parser.parse(RC_result)['Redundancy checker result']
        print("Redundancy checker result:", Is_Redundant)

        if (not Is_Test) and (Is_Unnecessary) and (not Is_Redundant):
            Supplementary_task_formatted = {"Task type": "supplementary task", "Content": Supplementary_Task, "Priority": overall_task_dict["Priority"]}
            Suggestion_Task_list.append(Supplementary_task_formatted)
            supplementary_task_finder_counter += 1
        else:
            Supplementary_task_checker_success = False
            supplementary_task_finder_counter += 1
            if supplementary_task_finder_counter >= supplementary_task_finder_counter_threshold:
                print("Checker is over due to over 2 times. Maybe more suggestions are needed.")

    Suggestion_Task_list = remove_duplicate_dicts(Suggestion_Task_list)

    print("Final checker module complete.")

    # Create dataPacket
    data_packet = create_data_packet(Decomposed_Task_list, robot_id)

    print("Decomposition complete. Data packet created.")
    print("Final Decomposed Task List:", Decomposed_Task_list)
    print("Suggestion Task List:", Suggestion_Task_list)

    return data_packet, Decomposed_Task_list, Suggestion_Task_list


if __name__ == "__main__":
    # Example usage
    overall_task_dict = {
        "Content": "Build a drainage pipe driven by a water pump from point A to point B, with the water pump installed at point B.",
        "Priority": 1
    }
    my_capability = "Install the water pump."
    #environmental_information = "There is a passable path from point M, which is the current location of the robot, to point B. The water pump and drainage pipe are stored at point C. The ground of the workspace is hard."
    
    environmental_information ={
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


    knowledge_input_list = """
    '''json
    {
    "Installation location of the water pump": string // Please fill in the coordinates or representative letter of the water pump installation location. If neither is explicitly specified, this item should be filled in as unknown.
    "Current location of the water pump": string // Please fill in the current coordinates or representative letter of the water pump. If neither is explicitly specified, this item should be filled in as unknown.
    "Current location of the robot": string // Please fill in the coordinates or representative letter of the robot's current location. If neither is explicitly specified, this item should be filled in as unknown.
    "Ground state": string // If the ground is soft, fill in soft. If the ground is hard, fill in hard. If neither is explicitly specified, this item should be filled in as unknown.
    }'''
    """
    knowledge_of_decomposition = """
    All tasks to be published should adhere to the following JSON format:
    '''json
    {
    "Task type": string // The type of task to be published. This type should be chosen from one of the following: preliminary task, collaborative task, subsequent task, independent task, or incremental task.
    "Content": string // The specific content of the task to be published.
    "Priority": int // if (the type of task to be published is a preliminary task) then [fill in %d], 
    elseif (the type of task to be published is a collaborative task, independent task, or incremental task) then [fill in %d], 
    elseif (the type of task to be published is a subsequent task) then [fill in %d].
    }'''
    """ % (overall_task_dict["Priority"] + 1, overall_task_dict["Priority"], overall_task_dict["Priority"] - 1)
    knowledge_template = """
    if (there is no passable path from the point {Current location of the robot} to the point {Installation location of the water pump})
    then [
    if ({Ground state} == hard) 
        then [publish preliminary task:  Clear a passable path from point {Current location of the robot} to the point {Installation location of the water pump}.]
    elseif ({Ground state} == soft) 
        then [publish preliminary task: Lay down a working path to create a specialized working path from point {Current location of the robot} to the point {Installation location of the water pump}.]
    ]

    if (the water pump is not at point {Installation location of the water pump})
    then [
    publish preliminary task: Transport the water pump from point {Current location of the water pump} to the point {Installation location of the water pump}.
    ]
    """
    robot_id = 'robot_1'
    start_from_scratch = True

    data_packet, decomposed_task_list, suggestion_task_list = decompose_task(
        overall_task_dict, 
        my_capability, 
        environmental_information, 
        knowledge_input_list, 
        knowledge_of_decomposition,
        knowledge_template,
        robot_id, 
        start_from_scratch
    )

    # Print the results
    print(json.dumps(data_packet, indent=2))
    print(decomposed_task_list)
    print(suggestion_task_list)
