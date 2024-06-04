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
