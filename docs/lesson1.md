# Controller Manager: Lesson


In this lesson, we will delve into the Controller_Manager class, which oversees the management of all controllers. The source code for this class can be found at Controllers/controller_manager.py. The primary role of this class is to integrate and coordinate modular controllers to facilitate complex robotic movements, ensuring the orderly execution of tasks. For example, this includes tasks such as grabbing a beaker, moving it to a specified position, and returning to the initial position after completing a pouring task. We will cover the key concepts, functions, and features of Controller_Manager, providing you with the knowledge needed to effectively utilize it in designing your own robotic movements.

The Controller_Manager class can add, execute, and retrieve specified controllers, and it can systematically combine common operations in chemical experiment scenarios based on a linear queue of controllers. We will explore these features in detail later in this lesson. Let's begin with an overview of the basic functions in Controller_Manager.

