# Sim Container： Lesson

In this instructional module, we delve into the intricacies of the Simcontainer class, which is an advanced extension of the Container class tailored for use within a simulation environment.  This specialized class has been designed to facilitate the integration of the Container entity into the simulation framework, thereby enabling the simulation of chemical processes within the Container.  The Simcontainer class not only encapsulates the fundamental functionalities of the Container class but also augments them with additional features that cater to the specific requirements of the simulation context.

As an integral component of the simulation environment, the Simcontainer class serves as the operational target for robotic arms, allowing for the execution of a plethora of tasks related to chemical experimentation.  This class provides a robust interface through which robotic agents can interact, ensuring a seamless integration of robotic manipulation with the simulated chemical processes.

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
