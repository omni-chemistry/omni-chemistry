# Controlller：Lesson

## Overview:
In this lesson, we will explore the _Controller class, which is fundamental to all robot operations and is utilized in Isaac-Sim for implementing robotic operations. Inspired by this class, Chemistry3D has developed several chemical-oriented controllers specifically designed for managing robot operations within chemical experiments. We will introduce the operation-oriented components of these controllers, detailing their code logic, functions, and properties. This will enable you to seamlessly integrate and utilize these components in your own Controller implementations.

If you need more detailed instructions and code, I suggest you refer to the official [**document**](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html) of Isaac-Sim.

## An example controlller

We will first start by importing the necessary required modules, both external and local. By now this step should seem very familiar as we have done them in both reaction and extraction lessons.

```python
import numpy as np
from omni.isaac.franka.controllers.rmpflow_controller import RMPFlowController
from Controllers.pick_move_controller import PickMoveController

pickmove_controller =PickMoveController(name="pickmove_controller",cspace_controller=RMPFlowController(name="pickmove_cspace_controller", robot_articulation=robot),gripper=robot.gripper)

actions = controller.forward(picking_position =  np.array([0.3,-0.3,0.1]),target_position = np.array([0.3,0.3,0.3]),current_joint_positions = robot.get_joint_positions())

robot.apply_action(actions)

```

    



