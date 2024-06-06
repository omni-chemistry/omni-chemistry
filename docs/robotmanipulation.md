
# Embodied Intelligence
![image](https://github.com/omni-chemistry/omni-chemistry/assets/171568986/f368d4b7-5afe-4af2-b3de-97f81edcd80e)


To evaluate the development capabilities of Chemistry3D in embodied intelligence tasks, we initially designed a chemical experiment scene. The laboratory setup included a table equipped with containers of (KMnO\textsubscript{4}) and (FeCl\textsubscript{2}), as well as two empty beakers. Within the overall framework, we constructed agents for robotic control. These agents were responsible for acquiring environmental information, generating robotic operation tasks, initializing different motion controllers, and managing robotic operations through the Controller Manager. The agents acquired information about the experimental scene, enabling the robot to observe interactive objects and generate potential chemical reactions based on its chemical reaction knowledge base. Subsequently, as shown in Figure above, we utilized natural language input to direct the robot to complete the relevant chemical experiment tasks.

In this lesson, we'll talk about our agent fabrication process and how it's deployed in Chemistry3D. In Chemistry3D, a class named Multi-Agent System (MAS) is responsible for managing all the agents. Let's first introduce the main functions in mas and their corresponding description.

## Event Function
| Function                          | Description                                                                                       |
|-----------------------------------|---------------------------------------------------------------------------------------------------|
| `__init__(self, world, controller_manager)` | Initializes the MAS class, setting the simulation world and controller manager.                |
| `agents_initialization(self)`     | Initializes agents for different tasks and loads their system prompts.                           |
| `_update_system_prompts(self)`    | Updates system prompts for all agents.                                                           |
| `_generate_plan(self, controllers_str)` | Generates a plan for a given task.                                                           |
| `_response_reaction(self, expected_chem)` | Responds to a chemical reaction, returning the response message.                               |
| `_add_particles(self)`            | Adds particles for each object.                                                                  |
| `_add_sim_container(self, particle_set_str)` | Adds simulation containers for each object.                                                 |
| `_add_rigidbody(self)`            | Adds rigidbody for each object.                                                                  |
| `_add_controllers(self, controllers_str)` | Adds controllers for a given task.                                                           |
| `_add_tasks(self, controllers_str)` | Adds tasks for a given controller.                                                            |
| `_generate_controllers(self, prompt, observation)` | Generates controllers for a given task.                                                      |
| `_observations_to_string(self, observation)` | Converts observation dictionary into a descriptive string.                                   |


## How to Write Prompts for the Multi-Agent System (MAS)

When working with the Multi-Agent System (MAS), it is essential to understand how to write effective prompts that guide the system to perform specific tasks. Here’s a detailed guide on composing prompts for MAS, including their structure and components. Here we will give an example of how to create an agent for adding controllers to controller manager.

### Components of a Prompt

A prompt for MAS typically consists of several key sections that ensure the system comprehends the task and performs it accurately. These components are:

1. **Task Description**: Clearly explain the overall task being accomplished by the controllers added in order.
2. **Code**: Provide the Python code required to add the controllers in the specified order. This code is written without line breaks and uses `\n` to represent line breaks.

**Task Description** is used to pass information across agents to ensure the consistency of the operation object, and **Code** is used to output the target code string. Here is an template in our prompt:

The output format for the prompt should be in JSON format as shown below:

```json
{   
    "Task Description": "What task is being accomplished by the controllers added in order",
    "Code": "Write Python code to add the controllers in order"
}
```

Rules for Writing Prompts：

1.Controller Names: The name of each controller should match the purpose of the current task.

2.Refer to Examples: Use examples and codes provided in the function descriptions to avoid bugs.

3.No Line Breaks: The "Code" field in the JSON file should not contain line breaks; use \n to represent line breaks.

4.Related Functions: Pay attention to related functions that often appear together.

5.Code Indentation: Write the code directly after \n without spaces to avoid unexpected indent errors.

6.Object Operations: Explicitly describe the operations to be performed on specific objects, such as grabbing or pouring a particular bottle.

7.Predefined Functions: Use only the predefined functions as provided.


### Example Prompt
Here is an example of how to write a prompt for adding controllers in a specific order:

The output format for the prompt should be in JSON format as shown below:

```json
{
    "Task Description": "Grabbing a bottle of medicine, pouring its contents into a beaker, and then returning the bottle to its original position.",
    "Code": "controller_manager.add_controller('pick_move_controller', PickMoveController())\ncontroller_manager.add_controller('pour_controller', PourController())\ncontroller_manager.add_controller('return_controller', ReturnController())"
}
```

Rules for Writing Prompts
1.Controller Names: The name of each controller should match the purpose of the current task.
2.Refer to Examples: Use examples and codes provided in the function descriptions to avoid bugs.
3.No Line Breaks: The "Code" field in the JSON file should not contain line breaks; use \n to represent line breaks.
4.Related Functions: Pay attention to related functions that often appear together.
5.Code Indentation: Write the code directly after \n without spaces to avoid unexpected indent errors.
6.Object Operations: Explicitly describe the operations to be performed on specific objects, such as grabbing or pouring a particular bottle.
7.Predefined Functions: Use only the predefined functions as provided.


Detailed Steps
Identify the Task: Clearly define what the task is and which objects are involved.
Write the Code: Using the examples and predefined functions, write the code to add the controllers in the correct sequence.
Format the Code: Ensure the code is formatted correctly with \n to represent line breaks and no additional spaces that might cause indentation errors.

To ensure the code outputs correctly, we need some predefined functions. You can create a predefined functions file to store all the functions and add any newly created functions to it. When importing the prompt into the agent, concatenate the prompt with the functions file to ensure the code outputs correctly.

```python
{
    # predefined_functions.py
    
    def add_controller(name, controller):
        controller_manager.add_controller(name, controller)
    
    class PickMoveController:
        # Define the PickMoveController class
        pass
    
    class PourController:
        # Define the PourController class
        pass
    
    class ReturnController:
        # Define the ReturnController class
        pass

}
```
## How to Use the Multi-Agent System (MAS) and Add Corresponding Agents
This tutorial explains how to use the Multi-Agent System (MAS) for managing and simulating chemical reactions and robotics control in a simulated environment. We will guide you through adding agents and using the MAS with appropriate prompts and predefined functions.

### Overview
The MAS is designed to facilitate the control of robotic tasks and chemical simulations by leveraging multiple agents that work together. Each agent is responsible for different aspects of the system, from generating controllers to handling reactions.
