# Controller Manager: Lesson


In this lesson, we will delve into the Controller_Manager class, which oversees the management of all controllers. The source code for this class can be found at Controllers/controller_manager.py. The primary role of this class is to integrate and coordinate modular controllers to facilitate complex robotic movements, ensuring the orderly execution of tasks. For example, this includes tasks such as grabbing a beaker, moving it to a specified position, and returning to the initial position after completing a pouring task. We will cover the key concepts, functions, and features of Controller_Manager, providing you with the knowledge needed to effectively utilize it in designing your own robotic movements.

The Controller_Manager class can add, execute, and retrieve specified controllers, and it can systematically combine common operations in chemical experiment scenarios based on a linear queue of controllers. We will explore these features in detail later in this lesson. Let's begin with an overview of the basic functions in Controller_Manager.

## Event Functions

| Function Name     | Description |
| ----------- | ----------- |
| __init__    | Initialize the ControllerManager with the simulation world, robotic arm, and gripper.       |
| add_controller   | 	Add a controller instance to the list of controllers.        |
| add_task   | 	Add a task to the list of tasks.        |
| get_current_controller_name   | Get the name of the current controller.       |
| get_current_controller   | Get the current controller instance.        |
| execute   | Execute the current task using the current controller.        |
| generate_task_params   | Generate task parameters based on a template and current observations.        |
| get_value_by_path   | Get a value from nested dictionaries based on a path.        |
| process_concentration_iters   | Process concentration iterators to print their next values and remove exhausted iterators.        |
| need_solid_melt   | Check if solid needs to be melted.        |
| set_need_solid_melt   | Set the flag indicating if solid needs to be melted.        |
| set_need_new_liquid   | Set the flag indicating if new liquid needs to be created in Sim_Container2.        |
| need_new_liquid   | Get the status of whether new liquid needs to be created.        |
| get_current_liquid_color   | Get the color of the new liquid from the current controller.        |
| reset   | Reset the task index and all controllers.        |
| is_done   | Check if all tasks have been completed.        |

The primary functionalities of the ControllerManager class that are crucial for robotic manipulation tasks are the add_controller, add_task, and execute methods. These methods are integral to defining the robot’s actions, including the specific movements it undertakes, the sequence of operations, and the overall workflow. For instance, in the context of a robotic arm performing grasping and pouring operations, these functions would be utilized to program the arm’s behavior. The add_controller method is employed to integrate a new controller into the system, which governs the robot’s movements. Subsequently, the add_task method is used to enqueue a new task into the robot’s queue of operations, specifying the type of movement and the parameters associated with it. Finally, the execute method is responsible for executing the current task, processing the observations from the environment, and sending appropriate signals to the robot’s actuators to perform the desired movements, such as gripping an object or tilting a container to pour its contents.

In the context of a robotic simulation environment, the task of grasping and pouring involves a series of predefined actions that are executed by a robotic arm. The following pseudocode outlines the steps required to perform these tasks using the ControllerManager class:

'''python
// Define the controllers for each action
pickmove_controller = new PickMoveController(name="pickmove_controller",cspace_controller=new RMPFlowController(name="pickmove_cspace_controller", robot_articulation=robot),gripper=robot.gripper)

pour_controller = new PourController(name="pour_controller",cspace_controller=new RMPFlowController(name="pour_cspace_controller", robot_articulation=robot),gripper=robot.gripper,Sim_Container1=Sim_Container1,Sim_Container2=self,pour_volume=pour_volume)


// Add the controllers to the controller manager
controller_manager.add_controller('pickmove_controller', pickmove_controller)
controller_manager.add_controller('pour_controller', pour_controller)


// Define the tasks with parameter templates
controller_manager.add_task("pick", {
    "picking_position": function(obs) { return obs[Sim_Container1.get_sim_container().name]["position"] },
    "target_position": function(obs) { return obs[Sim_Container1.get_sim_container().name]["Pour_Position"] },
    "current_joint_positions": function(obs) { return robot.get_joint_positions() },
})
controller_manager.add_task("pour", {
    'franka_art_controller': robot.get_articulation_controller(),
    "current_joint_positions": function(obs) { return robot.get_joint_positions() },
    'current_joint_velocities': function(obs) { return robot.get_joint_velocities() },
})

// Execute the tasks
while not controller_manager.is_done():
    controller_manager.execute(current_observations)
'''
| Function     | Parameter | Description |
| ----------- | ----------- | ----------- |
| add_controller    | controller_name       |A string identifier for the controller. This is used to reference the controller later in the code.       |
| add_controller   | controller_instance       |The actual instance of the controller object that will be managed by the ControllerManager. This object should implement the necessary methods for controlling the robot.       |
| add_task    | controller_type      |A string indicating the type of controller that should be used for this task. This is used to select the appropriate controller from the list when the task is executed.       |
| add_task   | 	param_template        |A dictionary that serves as a template for the parameters that will be passed to the controller when the task is executed. It can contain static values or functions that provide dynamic values based on the current state of the simulation.       |
