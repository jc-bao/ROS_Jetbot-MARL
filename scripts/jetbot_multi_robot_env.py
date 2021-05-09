#! /usr/bin/env python
# robot environment
import numpy
import rospy
from openai_ros import robot_gazebo_env
# TODO import msg type
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


# TODO[done] init robo env from gazebo env
class JetbotMultiRobotEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all Jetbot environments.
    """

    def __init__(self):
        """Initializes a new Multi-Robot environment.

        Args:
        """
        # Variables that we give through the constructor.
        # None in this case

        # Internal Vars
        self.max_step = 200
        self.step_cnt = 0
        # Robots number
        self.n = 2
        # TODO[done] add controler Hint: $ rosservice call /jetbot_0/controller_manager/list_controllers
        self.controllers_list = ['jetbot_joint_state_controller',
                                'jetbot_velocity_controller'
                                ]
        # TODO[done] add namespace Hint: $ rostopic list | grep controller
        self.robot_name_space = ["jetbot_"+str(i) for i in range(self.n)]

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(JetbotMultiRobotEnv, self).__init__(controllers_list=self.controllers_list,
                                                robot_name_space=self.robot_name_space,
                                                reset_controls=True)



        """
        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that th stream of data doesnt flow. This is for simulations
        that are pause for whatever the reason
        2) If the simulation was running already for some reason, we need to reset the controlers.
        This has to do with the fact that some plugins with tf, dont understand the reset of the simulation
        and need to be reseted to work properly.
        """
        self.gazebo.unpauseSim()
        if isinstance(self.controllers_object, list):
            for i in range(self.n):
                self.controllers_object[i].reset_controllers()
        else:
            self.controllers_object.reset_controllers()
        self._check_all_sensors_ready()

        # We Start all the ROS related Subscribers and publishers
        # TODO[done] add subscriber publisher
        for i in range(self.n):
            rospy.Subscriber("/" + self.robot_name_space[i] + "/joint_states", JointState, self._joints_callback, i)
            rospy.Subscriber("/" + self.robot_name_space[i] + "/jetbot_velocity_controller/odom", Odometry, self._odom_callback, i)
            rospy.Subscriber("/" + self.robot_name_space[i] + "/scan", LaserScan,
                             self._lidar_callback, i)

        self._vel_pub = [rospy.Publisher('/' + name + '/jetbot_velocity_controller/cmd_vel',
                                        Twist, queue_size=6) for name in self.robot_name_space] # ??? queue size

        self._check_publishers_connection()
        
        self.gazebo.pauseSim()

    # Methods needed by the RobotGazeboEnv
    # ----------------------------
    def step(self, action):
        """
        Function executed each time step.
        Here we get the action execute it in a time step and retrieve the
        observations generated by that action.
        :param action list:
        :return: obs, reward, done, info
        """

        """
        Here we should convert the action num to movement action, execute the action in the
        simulation and get the observations result of performing that action.
        """
        self.step_cnt += 1
        rospy.logdebug("START STEP OpenAIROS")
        self.gazebo.unpauseSim()
        self._set_action(action)
        self.gazebo.pauseSim()
        obs = self._get_obs()
        done = self._is_done(obs)
        reward = self._compute_reward(obs, done)
        info = {}
        if isinstance(reward, list):
            for i in range(len(reward)):
                self.cumulated_episode_reward += reward[i][0]
        else:
            self.cumulated_episode_reward += reward
        rospy.logdebug("END STEP OpenAIROS")
        return obs, reward, done, info

    # TODO[done] define check (remember to add sub functions)
    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        return True
        
    def _check_all_sensors_ready(self):
        self._check_joint_states_ready()
        self._check_odom_ready()
        self._check_lidar_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_joint_states_ready(self):
        self.joints = [None]*self.n
        #self.joints = []
        for i in range(self.n):
            while self.joints[i] is None and not rospy.is_shutdown():
                try:
                    self.joints[i] = rospy.wait_for_message("/" + self.robot_name_space[i] + "/joint_states", JointState, timeout=1.0)
                    rospy.logdebug("Current /" + self.robot_name_space[i] +"/joint_states READY=>" + str(self.joints[i]))
                except:
                    rospy.logerr("Current /" + self.robot_name_space[i] +"/joint_states not ready yet, retrying for getting joint_states")
        return self.joints

    def _check_odom_ready(self):
        self.odom = [None]*self.n
        #self.odom = []
        for i in range(self.n):
            while self.odom[i] is None and not rospy.is_shutdown():
                try:
                    self.odom[i] = rospy.wait_for_message("/" + self.robot_name_space[i] + "/jetbot_velocity_controller/odom", Odometry, timeout=1.0)
                    rospy.logdebug("Current /" + self.robot_name_space[i] +"/jetbot_velocity_controller/odom READY=>" + str(self.odom[i]))

                except:
                    rospy.logerr("Current /" + self.robot_name_space[i] +"/jetbot_velocity_controller/odom not ready yet, retrying for getting odom")

        return self.odom

    def _check_lidar_ready(self):
        self.lidar = [None]*self.n

        for i in range(self.n):
            while self.lidar[i] is None and not rospy.is_shutdown():
                try:
                    self.lidar[i] = rospy.wait_for_message("/" + self.robot_name_space[i] + "/scan", LaserScan, timeout=1.0)
                    rospy.logdebug("Current " + self.robot_name_space[i] +"/scan READY=>" + str(self.lidar[i]))

                except:
                    rospy.logerr("Current " + self.robot_name_space[i] +"/scan not ready yet, retrying for getting lidar")

        return self.lidar
    # ----------------------------

    
    # TODO[done] define callback / publisher
    def _joints_callback(self, data, i):
        self.joints[i] = data
    
    def _odom_callback(self, data, i):
        self.odom[i] = data

    def _lidar_callback(self, data, i):
        self.lidar[i] = data

    # TODO[done] define connection check function
    def _check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 1hz
        for i in range(self.n):
            while self._vel_pub[i].get_num_connections() == 0 and not rospy.is_shutdown():
                rospy.logdebug("No susbribers to _vel_pub  for agent " + str(i) + " yet so we wait and try again")
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    # This is to avoid error when world is rested, time when backwards.
                    pass
            rospy.logdebug("_vel_pub Publisher for agent " + str(i) + " Connected")

        rospy.logdebug("All Publishers READY")
    
    # Methods that the TaskEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TaskEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()
    
    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()
        
    # Methods that the TaskEnvironment will need.
    # ----------------------------
    # TODO[done] define how to move
    def move_joints(self, linear_vel, angular_vel):
        for i in range(self.n):
            vel_msg = Twist()
            vel_msg.linear.x = linear_vel[i]
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = angular_vel[i]
            rospy.logdebug("Jetbot Velocity for agent " + str(i) + ">>" + str(linear_vel[i])+','+str(angular_vel[i]))
            self._vel_pub[i].publish(vel_msg)
            # TODO[done] Change the desired velocity
        self.wait_until_jetbot_is_in_vel(linear_vel, angular_vel)
    
    # TODO[done] sub function of move_joint
    def wait_until_jetbot_is_in_vel(self, target_linear_vel, target_angular_vel):
        # ATT angular velocity not considered
        rate = rospy.Rate(10)
        start_wait_time = rospy.get_rostime().to_sec()
        end_wait_time = 0.0
        epsilon = 1
        while not rospy.is_shutdown():
            joint_data = self._check_joint_states_ready()
            are_close = []
            for i in range(self.n):
                if joint_data[i] is not None:
                    v_plus = target_linear_vel[i] + epsilon
                    v_minus = target_linear_vel[i] - epsilon
                    w_pulse = target_angular_vel[i] + epsilon
                    w_minus = target_angular_vel[i] - epsilon
                    real_linear_vel = (joint_data[i].velocity[0]+joint_data[i].velocity[1])
                    rospy.logdebug("VEL=" + str(real_linear_vel) + ", ?RANGE=[" + str(v_minus) + ","+str(v_plus)+"]")
                    are_close.append((real_linear_vel <= v_plus) and (real_linear_vel > v_minus))
            end_wait_time = rospy.get_rostime().to_sec()
            if end_wait_time - start_wait_time > 5.0:
                break
            if joint_data[0] is None or all(are_close):
                rospy.logdebug("Reached Velocity!")
                break
            rospy.logdebug("Not there yet, keep waiting...")
            rate.sleep()
        delta_time = end_wait_time - start_wait_time
        rospy.logdebug("[Wait Time=" + str(delta_time)+"]")
        return delta_time
        
    # TODO[done] define how to get state
    def get_joints(self):
        return self.joints
    
    def get_odom(self):
        return self.odom

    def get_laser(self):
        return self.lidar
