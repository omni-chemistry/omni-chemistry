# Sim Container： Lesson
![image](https://github.com/omni-chemistry/omni-chemistry/assets/171568986/d24c0843-bff3-4e53-92ce-d72dd79747c0)

## Introduction
In this tutorial, we will explore the Sim_Container class, which is an extension of the Container class. The Sim_Container class is designed for simulating chemical reactions and interactions within the Isaac-Sim environment. It integrates physical and chemical simulation capabilities, allowing for detailed manipulation and analysis of chemical processes.

## What is SimContainer?
Sim_Container is a specialized class that inherits from the Container class, designed to handle chemical reactions within a simulated environment. This class not only manages solute information, volume, and temperature but also integrates with a physics simulator to represent physical properties and behaviors of chemical substances.

## An example init Sim_Container：
To provide an example code for initialization, we first need to understand the parameters in the Sim_Container class and what they mean. 

```python

# Create an instance of the Sim_Container class

world = World()
sim_container = world.scene.get_object("container_object")

sim_container_instance = Sim_Container(
    world=world,
    sim_container=sim_container,
    solute={'solute_1': 10, 'solute_2': 20},  # Example solute dictionary
    org=True,  # Indicates the container is organic
    volume=10,  # Volume of the container in liters
    temp=25,  # Temperature of the container in degrees Celsius
    verbose=False  # Indicates not to output detailed information
)
```
Here is a table and description of the parameters in the initialization.

|Parameter    | Type | Description |
| ----------- | ----------- |----------- |
| world    | World      |The World object containing the simulation environment.    |
| sim_container   | 	 xform    |The object representing the container in the simulation environment.     |
| solute   | 	dict        |A dictionary containing solute substances and their quantities.      |
| org   |bool       |A flag indicating whether the container is organic.     |
| volume   | float     |The volume of the container in liters. |
| temp	   | float        |The temperature of the container in degrees Celsius.      |
| verbose		   | bool        |A flag indicating whether to output detailed information.      |

In this example, we create an instance of the Sim_Container class named sim_container_instance, which references a container entity named container_object in the simulation environment. We also set some solutes, specify that the container is organic, and define its volume and temperature. Finally, we set verbose to False, which means detailed information will not be output during initialization.

## Event Function：

| Function Name       | Description                                                                                       |
|---------------------|---------------------------------------------------------------------------------------------------|
| `__init__`          | Initialize the Sim_Container with the world, sim_container, object, solute, volume, temperature, and verbosity.|
| `get_color`         | Retrieve the color based on the organic property and solute information.                           |
| `sim_update`        | Update simulation with the new state and task details for the controller.                          |
| `set_sim_container` | Set the simulation container.                                                                     |
| `get_sim_container` | Get the current simulation container.                                                             |
| `set_object`        | Update the object dictionary.                                                                     |
| `get_object`        | Get the current object dictionary.                                                                |
| `add_liquid`        | Add a liquid to the liquid list.                                                                  |
| `remove_solid`      | Remove a solid from the solid list.                                                               |
| `melt_solid`        | Melt and remove solid objects.                                                                    |
| `create_liquid`     | Create a new liquid in the simulation.                                                            |
| `remove_liquid`     | Remove a liquid from the liquid list if it exists.                                                |
| `add_solid`         | Add a solid to the solid list.                                                                    |
| `remove_solid`      | Remove a solid from the solid list if it exists.                                                    |

### Method Descriptions

#### '''get_color()'''

The `get_color` method determines the color of the container's content based on whether it is organic or inorganic, and its solute information. This is useful for visualizing the contents of the container in the simulation environment.

#### '''sim_update()'''

The `sim_update` method updates the simulation state with new parameters and tasks for the controllers. It configures tasks like picking, pouring, and returning containers, and integrates these tasks with the robotic controllers in Isaac-Sim.

#### '''set_sim_container()'''

This method sets the simulation container to a new container, allowing for dynamic changes in the simulation setup.

#### '''get_sim_container()'''

This method retrieves the current simulation container, providing access to the container's properties and state.

#### '''set_object()'''

The `set_object` method updates the object dictionary, which keeps track of the liquid and solid objects in the simulation.

#### '''get_object()'''

This method retrieves the current object dictionary, providing information about the liquid and solid objects in the container.

#### '''add_liquid()'''

The `add_liquid` method adds a liquid to the liquid list in the object dictionary, enabling the simulation of adding liquids to the container.

#### '''remove_solid()'''

This method removes a solid from the solid list in the object dictionary, simulating the removal of solid objects from the container.

#### '''melt_solid()'''

The `melt_solid` method melts and removes solid objects, simulating the process of solids melting into liquids.

#### '''create_liquid()'''

The `create_liquid` method creates a new liquid in the simulation, adding it to the liquid list in the object dictionary.

#### '''remove_liquid()'''

This method removes a liquid from the liquid list if it exists, simulating the removal of liquids from the container.

#### '''add_solid()'''

The `add_solid` method adds a solid to the solid list in the object dictionary, simulating the addition of solid objects to the container.


## SimContainer Example in Isaac Sim

This example demonstrates the use of the `Sim_Container` class within the Isaac Sim environment. The script initializes a simulation world, sets up chemical containers with specific solutes, and manages their interactions using robotic controllers. The example aims to simulate chemical reactions and visualize the process, ultimately creating a video of the simulation. Below is the annotated code for the example:

### Initialize Isaac Sim

```python
# Launch Isaac Sim before any other imports
# These are the default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
```

The first two lines initialize the Isaac Sim application. The `SimulationApp` class is configured to run in non-headless mode, allowing for graphical output.

### Import Necessary Libraries

```python
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from Chemistry3D_Demo.Chemistry3D_Task import Chem_Lab_Task
from omni.isaac.franka import Franka
from omni.isaac.core.utils.types import ArticulationAction
from pxr import Sdf, Gf, UsdPhysics
from omni.isaac.franka.controllers.rmpflow_controller import RMPFlowController
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.physx.scripts import physicsUtils, particleUtils
from Controllers.Controller_Manager import ControllerManager
from Sim_Container import Sim_Container
from utils import Utils
```

The necessary libraries and modules are imported, including numpy, Isaac Sim core modules, and custom controllers and utilities.

### Initialize the Simulation World

```python
# Initialize the simulation world
my_world = World(physics_dt=1.0 / 120.0, stage_units_in_meters=1.0, set_defaults=False)
my_world._physics_context.enable_gpu_dynamics(flag=True)
stage = my_world.scene.stage
scenePath = Sdf.Path("/physicsScene")
utils = Utils()
utils._set_particle_parameter(my_world, particleContactOffset=0.003)
```

The simulation world is initialized with a physics time step of 1/120 seconds and stage units set to meters. GPU dynamics are enabled for better performance.

### Add the Chemical Lab Task

```python
# Add the chemical lab task to the simulation world
my_world.add_task(Chem_Lab_Task(name='Chem_Lab_Task'))
my_world.reset()
```

The chemical lab task is added to the simulation world, and the world is reset to its initial state.

### Retrieve Objects from the Scene

```python
# Retrieve objects from the scene
Franka0 = my_world.scene.get_object("Franka0")
current_observations = my_world.get_observations()
controller_manager = ControllerManager(my_world, Franka0, Franka0.gripper)
```

Robotic objects (Franka0) and a camera are retrieved from the scene. The `ControllerManager` is initialized to manage the robotic controllers.

### Initialize Simulation Containers

```python
# Initialize simulation containers with specific properties
Sim_Bottle1 = Sim_Container(
    world=my_world,
    sim_container=my_world.scene.get_object("Bottle1"),
    solute={'c1ccc2cc3ccccc3cc2c1': 10},
    org=True,
    volume=10
)
Sim_Bottle2 = Sim_Container(
    world=my_world,
    sim_container=my_world.scene.get_object("Bottle2"),
    solute={'BrBr': 20},
    org=True,
    volume=10
)

Sim_Bottle1 = Sim_Container(world = my_world, sim_container = my_world.scene.get_object("Bottle1"), solute={'MnO4^-': 0.02, 'K^+': 0.02,'H^+': 0.04,'SO4^2-': 0.02}, volume=0.02)
Sim_Bottle2 = Sim_Container(world = my_world, sim_container = my_world.scene.get_object("Bottle2"), solute={'Fe^2+': 0.06, 'Cl^-': 0.12}, volume=0.02)
Sim_Beaker1 = Sim_Container(world = my_world,sim_container = my_world.scene.get_object("Beaker1"))
Sim_Beaker2 = Sim_Container(world = my_world,sim_container = my_world.scene.get_object("Beaker2"))

Sim_Beaker1.sim_update(Sim_Bottle1,Franka0,controller_manager)
Sim_Beaker2.sim_update(Sim_Bottle2,Franka0,controller_manager)
Sim_Beaker2.sim_update(Sim_Beaker1,Franka0,controller_manager)
```

Simulation containers (`Sim_Container`) are initialized with specific solutes and volumes. These containers represent chemical substances in the simulation.

### Main Simulation Loop

```python
# Main simulation loop
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            controller_manager.reset()
        current_observations = my_world.get_observations()
        controller_manager.execute(current_observations=current_observations)
        controller_manager.process_concentration_iters()
        if controller_manager.need_new_liquid():
            controller_manager.get_current_controller()._get_sim_container2().create_liquid(controller_manager, current_observations)
        if controller_manager.need_solid_melt():
            controller_manager.get_current_controller()._get_sim_container2().melt_solid(controller_manager)
        if controller_manager.is_done():
            my_world.pause()
# Close the simulation application
simulation_app.close()
```

This loop runs the simulation, updates the world state, and manages the tasks of the controllers. Images are captured and saved at regular intervals, and a video is generated from these images once the simulation is complete.




