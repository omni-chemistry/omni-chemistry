# Chemistry3D

> **Welcome to Chemistry3D’s documentation!**
>
> Visit our website: [Chemistry3D](https://sites.google.com/view/chemistry3d)
>
> Read our paper: [arXiv:2406.08160](https://arxiv.org/abs/2406.08160)

Chemistry3D is an advanced chemistry simulation laboratory leveraging the capabilities of IsaacSim. This platform offers a comprehensive suite of both organic and inorganic chemical experiments, which can be conducted in a simulated environment. One of the distinguishing features of Chemistry3D is its ability to facilitate robotic operations within these experiments. The primary objective of this simulation environment is to provide a robust and reliable testing ground for both experimental procedures and the application of robotics within chemical laboratory settings.



The development of this system aims to integrate chemical experiment simulation with robotic simulation environments, thereby enabling the visualization of chemical experiments while facilitating the simulation of realistic robotic operations. This environment supports the advancement of embodied intelligence for robots, the execution of reinforcement learning tasks, and other robotic operations. Through this integrated environment, we aspire to seamlessly combine chemical experiment simulation with robotic simulation, ultimately creating a comprehensive test environment for the entire chemical experimentation process.

## What can we do in Chemistry3D?
* **Multiple chemical reaction simulation**
* Chemistry can simulate a range of organic and inorganic reactions.

* **Robotic manipulaitions**
*  In Chemistry3D, we have modularized the motion controllers for operations frequently encountered in chemistry experiments. A Controller Manager class has been established to manage these controllers and facilitate the integration of each module. Additionally, we support users in creating new controllers tailored to their specific needs, with the Controller Manager handling the management of these newly created controllers.

* **Visual sim2real**
* The application of robotic manipulation driven by visual inputs spans numerous domains. Chemistry3D, developed using Isaac-Sim, leverages ultra-realistic light simulation capabilities. For our visual task objects, we selected transparent chemical containers, commonly used in chemical experiment scenarios, and successfully implemented sim-to-real transfer for these transparent objects. To construct a visual training benchmark, we integrated segmentation_models.pytorch within Chemistry3D. Experimental results demonstrate that sim-to-real transfer for complex light-simulated objects is achievable in Chemistry3D. Additionally, this environment provides an open platform for testing various algorithms.

* **Reinforcement learning**
* Chemistry3D incorporates reinforcement learning for robots utilizing OmniIsaacGymEnvs within Isaac-Sim. This integration provides a diverse array of user-defined tasks, facilitating rapid advancement in reinforcement learning research.

* **Embodied intelligence**
* In Chemistry3D, we have introduced multiple agents designed for generating and planning motion tasks. Users have the capability to create new agents and define prompts to accomplish specific tasks according to their requirements.

