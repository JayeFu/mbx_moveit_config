# Introduction
This is a moveit config package for mbx. It will be used as the back-end planner for mbx.

# TIPS
1. Do not worrry. I have moved all the necessary model files into this package itself. You do not need `gazebo_example` for compilation.
2. relation between gazebo and moveit controllers
   1. See this [post](http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/controller_configuration/controller_configuration_tutorial.html) and this [post](https://answers.ros.org/question/312534/confused-about-loading-ros_control-and-moveit-controllers/)
   2. What really matters is that something like `/arm_controller/follow_joint_trajectory/goal` should be published by `move_group` and subscribed by `gazebo` at the same time
   3. and thus we need to create a new `yaml` file which contains both `joint_state_publisher` and `arm_controller`. Just like `config/gazebo_controllers.yaml`.