#! /usr/bin/env python
# task environment
import rospy
import numpy as np
import math
from gym import spaces
# TODO[done] import your robot env
import jetbot_robot_env
from gym.envs.registration import register
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion

max_episode_steps = rospy.get_param('/jetbot_0/max_episode_steps')

# TODO[done] register your env
register(
        id='JetbotTaskEnv-v0',
        entry_point='jetbot_task_env:JetbotTaskEnv',
        max_episode_steps=max_episode_steps,
    )

class JetbotTaskEnv(jetbot_robot_env.JetbotRobotEnv):
    def __init__(self):
        
        # TODO[done] action space setup
        # Only variable needed to be set here
        number_actions = rospy.get_param('/jetbot_0/n_actions')
        self.action_space = spaces.Discrete(number_actions)
    
        # Actions and Observations
        # TODO[done] set observation space setup
        self.linear_speed_fixed_value = rospy.get_param('/jetbot_0/linear_speed_fixed_value')
        self.angular_speed_fixed_value = rospy.get_param('/jetbot_0/angular_speed_fixed_value')
        self.max_x = rospy.get_param('/jetbot_0/max_x')
        self.max_y = rospy.get_param('/jetbot_0/max_y')
        number_observations = rospy.get_param('/jetbot_0/n_observations')
        """
        We set the Observation space for the 6 observations
        jetbot_observations = [
            round(current_jetbot_pos_x, 1),
            round(current_jetbot_pos_y, 1),
            round(yaw, 1),
        ]
        """

        max_yaw = math.pi
        high = np.array([
            self.max_x,
            self.max_y,
            max_yaw
            ])
        
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        
        rospy.logwarn("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logwarn("OBSERVATION SPACES TYPE===>"+str(self.observation_space))
        
        # TODO[done] set init state
        # Variables that we retrieve through the param server, loded when launch training launch.
        self.init_linear_vel = rospy.get_param("/jetbot_0/init_linear_vel")
        self.init_angular_vel = rospy.get_param("/jetbot_0/init_angular_vel")
        self.start_point = Point()
        self.start_point.x = rospy.get_param("/jetbot_0/init_pose/x")
        self.start_point.y = rospy.get_param("/jetbot_0/init_pose/y")
        self.start_point.z = rospy.get_param("/jetbot_0/init_pose/z")
        
        # TODO[done] set Rewards weight
        self.x_weight = rospy.get_param("/jetbot_0/x_weight")
        self.y_weight = rospy.get_param("/jetbot_0/y_weight")
        self.yaw_weight = rospy.get_param("/jetbot_0/yaw_weight")

        self.cumulated_steps = 0.0

        # TODO[done] init your **task!** env (change name)
        # Here we will add any init functions prior to starting the MyRobotEnv
        super(JetbotTaskEnv, self).__init__()

    # TODO set virtual functions
    '''
    init position
    init environment 
    set action RL action-> robot action
    obs
    reward
    done : force to quit env
    '''
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_joints(self.init_linear_vel,self.init_angular_vel)

        return True


    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        self.total_distance_moved = 0.0
        self.linear_speed = rospy.get_param('/jetbot_0/init_linear_vel')
        self.angular_speed = rospy.get_param('/jetbot_0/init_angular_vel')
        # For Info Purposes
        self.cumulated_reward = 0.0

    def _set_action(self, action):

        # We convert the actions to speed movements to send to the parent class Jetbot_Robot_Env
        if action == 0:# Stop
            self.linear_speed = 0
            self.angular_speed = 0
        elif action == 1:# Move Jetbot Forward
            self.linear_speed = self.linear_speed_fixed_value
            self.angular_speed = 0
        elif action == 2:# Turn Left
            self.linear_speed = 0
            self.angular_speed = self.angular_speed_fixed_value
        elif action == 3:# Increment Speed
            self.linear_speed = 0
            self.angular_speed = -self.angular_speed_fixed_value

        # We tell the Jetbot to move in this speed
        self.move_joints(self.linear_speed, self.angular_speed)

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        Find in The Doc you use
        :return:
        """

        # We get the orientation of the cube in RPY
        roll, pitch, yaw = self.get_orientation_euler()

        # We get the pos from the origin
        pos_x, pos_y, pos_z = self.get_pos()

        # We get the current speed of the Jetbot
        '''
        we do not use the speed currently
        '''

        jetbot_observations = np.array([
            round(pos_x, 1),
            round(pos_y, 1),
            round(yaw, 1),
        ])
        
        rospy.logdebug("Observations==>"+str(jetbot_observations))

        return jetbot_observations
        

    def _is_done(self, observations):

        pos_x = observations[0]
        pos_y = observations[1]

        if abs(pos_x) > self.max_x:
            rospy.logerr("WRONG pos_x==>" + str(pos_x))
            done = True
        elif abs(pos_y) > self.max_y:
            rospy.logerr("WRONG pos_y==>" + str(pos_y))
            done = True
        else:
            done = False

        return done

    def _compute_reward(self, observations, done):

        reward = 0

        if not done:

            # make x as large as possible
            pos_x = observations[0]
            rospy.logdebug("pos_x=" + str(pos_x))
            reward_x = abs(pos_x) * self.x_weight
            rospy.logdebug("reward_x=" + str(reward_x))

            # make y as small as possible
            pos_y = observations[1]
            rospy.logdebug("pos_y=" + str(pos_y))
            reward_y = -abs(pos_y) * self.y_weight
            rospy.logdebug("reward_y=" + str(reward_y))

            # Negative Reward for yaw different from zero.
            # make yaw as close to 0 as possible
            yaw = observations[2]
            rospy.logdebug("yaw=" + str(yaw))
            reward_yaw = -1 * abs(yaw) * self.yaw_weight
            rospy.logdebug("reward_yaw=" + str(reward_yaw))

            reward = reward_x + reward_y + reward_yaw

        else:
            reward = -1*self.end_episode_points


        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))
        
        return reward


    # Private functions
    # Internal TaskEnv Methods
    # all for _get_obs
    
    def get_orientation_euler(self):
        # We convert from quaternions to euler
        orientation_list = [self.odom.pose.pose.orientation.x,
                            self.odom.pose.pose.orientation.y,
                            self.odom.pose.pose.orientation.z,
                            self.odom.pose.pose.orientation.w]
    
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw

    def get_pos(self):
        return self.odom.pose.pose.position.x,self.odom.pose.pose.position.y,self.odom.pose.pose.position.z
