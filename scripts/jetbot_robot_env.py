#! /usr/bin/env python
# robot environment
import numpy
import rospy
from openai_ros import robot_gazebo_env
# TODO impoer msg type
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


# TODO[done] init robo env from gazebo env
class JetbotRobotEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all CubeSingleDisk environments.
    """

    def __init__(self):
        """Initializes a new CubeSingleDisk environment.

        Args:
        """
        # Variables that we give through the constructor.
        # None in this case

        # Internal Vars
        # TODO[done] add controler Hint: $ rosservice call /jetbot_0/controller_manager/list_controllers
        self.controllers_list = ['jetbot_joint_state_controller',
                                 'jetbot_velocity_controller'
                                 ]
        # TODO[done] add name spacr Hint: $ rostopic list | grep controller
        self.robot_name_space = "jetbot_0"

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(MyCubeSingleDiskEnv, self).__init__(controllers_list=self.controllers_list,
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
        self.controllers_object.reset_controllers()
        self._check_all_sensors_ready()

        # We Start all the ROS related Subscribers and publishers
        # TODO[done] add subscriber publisher
        rospy.Subscriber("/jetbot_0/joint_states", JointState, self._joints_callback)
        rospy.Subscriber("/jetbot_0/jetbot_velocity_controller/odom", Odometry, self._odom_callback)

        self._vel_pub = rospy.Publisher('/jetbot_0/jetbot_velocity_controller/cmd_vel',
                                             geometry_msgs/Twist, queue_size=1) # ??? queue size

        self._check_publishers_connection()

        self.gazebo.pauseSim()

    # Methods needed by the RobotGazeboEnv
    # ----------------------------
    
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
        rospy.logdebug("ALL SENSORS READY")

    def _check_joint_states_ready(self):
        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message("/jetbot_0/joint_states", JointState, timeout=1.0)
                rospy.logdebug("Current jetbot_0/joint_states READY=>" + str(self.joints))

            except:
                rospy.logerr("Current /jetbot_0/joint_states not ready yet, retrying for getting joint_states")
        return self.joints

    def _check_odom_ready(self):
        self.odom = None
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message("/jetbot_0/jetbot_velocity_controller/odom", Odometry, timeout=1.0)
                rospy.logdebug("Current /jetbot_0/jetbot_velocity_controller/odom READY=>" + str(self.odom))

            except:
                rospy.logerr("Current /jetbot_0/jetbot_velocity_controller/odom not ready yet, retrying for getting odom")

        return self.odom


    # CubeSingleDiskEnv virtual methods
    # ----------------------------

    
    # TODO[done] define callback / publisher
    def _joints_callback(self, data):
        self.joints = data
    
    def _odom_callback(self, data):
        self.odom = data
        
    # TODO[done] define connection check function
    def _check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while self._vel_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to _vel_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_vel_pub Publisher Connected")

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
        vel_msg = Twist()
        vel_msg.linear.x = linear_vel
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angular_vel
        rospy.logdebug("Jetbot Velocity>>" + str(linear_vel)+','+str(angular_vel))
        self._vel_pub.publish(vel_msg)
        self.wait_until_jetbot_is_in_vel(joint_speed_value.data)
    
    # TODO[done] sub function of move_joint
    def wait_until_jetbot_is_in_vel(self, target_linear_vel, target_angular_vel):
        # ATT angular velocity not considered
        rate = rospy.Rate(10)
        start_wait_time = rospy.get_rostime().to_sec()
        end_wait_time = 0.0
        epsilon = 0.1
        v_plus = target_linear_vel + epsilon
        v_minus = target_linear_vel - epsilon
        w_pulse = target_angular_vel + epsilon
        w_minus = target_angular_vel - epsilon
        while not rospy.is_shutdown():
            joint_data = self._check_joint_states_ready()
            real_linear_vel = (joint_data.velocity[0]+joint_data.velocity[1])
            rospy.logdebug("VEL=" + str(real_linear_vel) + ", ?RANGE=[" + str(v_minus) + ","+str(v_plus)+"]")
            are_close = (real_linear_vel <= v_plus) and (roll_vel > v_minus)
            if are_close:
                rospy.logdebug("Reached Velocity!")
                end_wait_time = rospy.get_rostime().to_sec()
                break
            rospy.logdebug("Not there yet, keep waiting...")
            rate.sleep()
        delta_time = end_wait_time- start_wait_time
        rospy.logdebug("[Wait Time=" + str(delta_time)+"]")
        return delta_time
        
    # TODO[done] define how to get state
    def get_joints(self):
        return self.joints
    
    def get_odom(self):
        return self.odom