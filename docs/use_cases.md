# Use Cases

#### 1. Develop your own RL Task

> Note: All staff you need to change in the code is commented by `TODO` in code. 

1. Robot Environment `Jetbot_robot_env.py`

   * The thing you need to do in this file:

     * Open Gazebo
     * Determine how to control
     * Determine which sensor to use
     * Set virtual method for Task Environment

   * Details

     * Init robot envrionment from gazebo environment.

       ```python
       class JetbotRobotEnv(robot_gazebo_env.RobotGazeboEnv):
       ```

     * Choose your own controller

       ```python
       self.controllers_list = ['jetbot_joint_state_controller',
                                       'jetbot_velocity_controller'
                                       ]
       ```

       > Hint: you can use `$ rosservice call /jetbot_0/controller_manager/list_controllers`to get the controller

     * Change the namespace

       ```
       self.robot_name_space = "jetbot_0"
       ```

       > Hint: use `$ rostopic list | grep controller*` to get the namespace

     * 

#### 2. Start training

```shell
roslaunch jetbot_rl start_training.launch
```

#### 2. Plot Training Result

Run rqt_multipolt

```shell
rosrun rqt_multiplot rqt_multiplot
```

Choose configure -> choose topic [openai/reward] -> choose start plot 

