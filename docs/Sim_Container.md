# Sim Container： Lesson
## Introduction
In this tutorial, we will explore the Sim_Container class, which is an extension of the Container class. The Sim_Container class is designed for simulating chemical reactions and interactions within the Isaac-Sim environment. It integrates physical and chemical simulation capabilities, allowing for detailed manipulation and analysis of chemical processes.

## What is SimContainer?
Sim_Container is a specialized class that inherits from the Container class, designed to handle chemical reactions within a simulated environment. This class not only manages solute information, volume, and temperature but also integrates with a physics simulator to represent physical properties and behaviors of chemical substances.

## An example Sim_Container：
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
| `remove_solid`      | Remove a solid from the solid list if it exists.     
