
# Embodied Intelligence
![image](https://github.com/omni-chemistry/omni-chemistry/assets/171568986/f368d4b7-5afe-4af2-b3de-97f81edcd80e)


To evaluate the development capabilities of Chemistry3D in embodied intelligence tasks, we initially designed a chemical experiment scene. The laboratory setup included a table equipped with containers of (KMnO\textsubscript{4}) and (FeCl\textsubscript{2}), as well as two empty beakers. Within the overall framework, we constructed agents for robotic control. These agents were responsible for acquiring environmental information, generating robotic operation tasks, initializing different motion controllers, and managing robotic operations through the Controller Manager. The agents acquired information about the experimental scene, enabling the robot to observe interactive objects and generate potential chemical reactions based on its chemical reaction knowledge base. Subsequently, as shown in Figure above, we utilized natural language input to direct the robot to complete the relevant chemical experiment tasks.

In this lesson, we'll talk about our agent fabrication process and how it's deployed in Chemistry3D

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


## Input

The input to the reaction bench is initialized in the `reaction_bench_v1.py` file. 

```python
class GeneralWurtzReact_v2(GenBench):
    """
    Class to define an environment which performs a Wurtz extraction on materials in a vessel.
    """
    metadata = {
        "render_modes": ["rgb_array"],
        "render_fps": 10,
    }
    def __init__(self):
        r_rew = RewardGenerator(use_purity=False,exclude_solvents=False,include_dissolved=False)
        shelf = Shelf([
            get_mat("diethyl ether",4,"Reaction Vessel"),
            get_mat("1-chlorohexane",1),
            get_mat("2-chlorohexane",1),
            get_mat("3-chlorohexane",1),
            get_mat("Na",3),
        ])
        actions = [
            Action([0],    [ContinuousParam(156,307,0,(500,))],  'heat contact',   [0],  0.01,  False),
            Action([1],    [ContinuousParam(0,1,1e-3,())],      'pour by percent',  [0],   0.01,   False),
            Action([2],    [ContinuousParam(0,1,1e-3,())],      'pour by percent',  [0],   0.01,   False),
            Action([3],    [ContinuousParam(0,1,1e-3,())],      'pour by percent',  [0],   0.01,   False),
            Action([4],    [ContinuousParam(0,1,1e-3,())],      'pour by percent',  [0],   0.01,   False),
        ]

        react_info = ReactInfo.from_json(REACTION_PATH+"/chloro_wurtz.json")
        
        super(GeneralWurtzReact_v2, self).__init__(
            shelf,
            actions,
            ["PVT","spectra","targets"],
            targets=react_info.PRODUCTS,
            default_events = (Event("react", (Reaction(react_info),), None),),
            reward_function=r_rew,
            discrete=False,
            max_steps=20
        )
```

In here we pass parameters such as the materials and solutes needed for the experiment, the path to an input vessel 
(if we're including one), the output vessel path, the number of time steps to be taken during each action, the amount
of time taken in each time step, and an indication to show if spectral plots show overlapping. 

We also pass in a reaction event which is constructed using a json file located in `chemistrylab/reactions/available_reactions`. This file includes important values that the engine will use to simulate the reaction. It contains information on the reactants, products, and solutes available to the reaction. Additionally, it includes arrays for the activation energy, stoichiometric coefficients, and concentration calculation coefficients. 

## Output

Once the reaction bench is reset and the render function is called, plots will appear showing data about the reaction 
being performed by the agent.

- Render
    - Plots thermodynamic variables and spectral data. The human render plots a minimal amount of data and provides a 
    'surface-level' understanding of the information portrayed.
    - Plots absorbance, time, temperature, volume, pressure, and the amount of reactants remaining.
  
![human render output](tutorial_figures/reaction/human_render_reaction.png)
