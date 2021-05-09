#! /usr/bin/env python
# task environment
import os
import rospy
import numpy as np
import math
from operator import itemgetter
from gym import spaces
# TODO[done] import your robot env
import jetbot_multi_robot_env
from gym.envs.registration import register
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf

# TODO[done] register your env
register(
        id='JetbotFormationEnv-v0',
        entry_point='jetbot_formation_env:JetbotFormationEnv',
    )

class JetbotFormationEnv(jetbot_multi_robot_env.JetbotMultiRobotEnv):
    def __init__(self, port=0):
        self.port = port
        os.environ['ROS_MASTER_URI'] = "http://localhost:1135" + str(self.port) + '/'
        rospy.init_node('jetbot_train_gym', anonymous=True, log_level=rospy.WARN)
        self.n = 2
        # TODO[done] action space setup
        # Only variable needed to be set here
        number_actions = 2
        #self.action_space = spaces.Discrete(number_actions)
        self.action_space = [spaces.Box(low=-1, high=1, shape=(number_actions,), dtype=np.float32) for _ in range(self.n)]
        self.observation_space = [spaces.Box(-np.inf, np.inf, shape=(2 * (self.n - 1) + 1 + 4,), dtype=np.float32) for _
                                  in range(self.n)]
        self.share_observation_space = []
        self.robot_name_space = ["jetbot_" + str(i) for i in range(self.n)]

        # Actions and Observations
        # TODO[done] set observation space setup
        self.linear_speed_fixed_value = 1.0
        self.angular_speed_fixed_value = 1.0
        self.max_x = 1000.0
        self.max_y = 1000.0
        max_yaw = math.pi
        rospy.logwarn("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logwarn("OBSERVATION SPACES TYPE===>"+str(self.observation_space))
        
        # TODO[done] set init state
        # Variables that we retrieve through the param server, loded when launch training launch.
        self.init_linear_vel = [1.0 for name in self.robot_name_space]
        self.init_angular_vel = [0.0 for name in self.robot_name_space]

        self.start_point = Point()
        x = [-1.5, 1.5, 0.0, 0.0]
        y = [0.0, 0.0, 1.5, -1.5]
        init_pos = [{"x":0.0, "y":0.0, "z":0.0, "yaw":0.0}, {"x":x[self.port], "y":y[self.port], "z":0.0, "yaw":0.0}]
        self.start_point.x = [init_pos[i]["x"] for i in range(len(init_pos))]
        self.start_point.y = [init_pos[i]["y"] for i in range(len(init_pos))]
        self.start_point.z = [init_pos[i]["z"] for i in range(len(init_pos))]
        self.start_yaw = [init_pos[i]["yaw"] for i in range(len(init_pos))]

        self.start_point_offset = Point()
        self.start_point_offset.x = [init_pos[i]["x"] for i in range(len(init_pos))]
        self.start_point_offset.y = [init_pos[i]["y"] for i in range(len(init_pos))]
        self.start_point_offset.z = [init_pos[i]["z"] for i in range(len(init_pos))]
        self.start_yaw_offset = [init_pos[i]["yaw"] for i in range(len(init_pos))]

        self.scan_map = [{} for _ in range(self.n)]

        self.cumulated_steps = 0.0

        # TODO[done] init your **task!** env (change name)
        # Here we will add any init functions prior to starting the MyRobotEnv
        super(JetbotFormationEnv, self).__init__()

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
        """Sets the Robots in their init pose
        """
        self.move_joints(self.init_linear_vel, self.init_angular_vel)

        return True

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        self.step_cnt = 0
        self.total_distance_moved = [0.0] * self.n
        self.linear_speed = [1.0 for name in self.robot_name_space]
        self.angular_speed = [0.0 for name in self.robot_name_space]
        self.scan_map = [{} for _ in range(self.n)]
        for i in range(self.n):
            x_offset, y_offset, z_offset = self.get_pos_offset(i)
            _, _, yaw_offset = self.get_orientation_euler_offset(i)
            self.start_point_offset.x[i] = x_offset
            self.start_point_offset.y[i] = y_offset
            self.start_point_offset.z[i] = z_offset
            self.start_yaw_offset[i] = yaw_offset

        # For Info Purposes
        self.cumulated_reward = [0.0] * self.n

    def _set_action(self, action):
        '''
        inputs: actions for N agents
        '''
        # We convert the actions to speed movements to send to the parent class Jetbot_Robot_Env
        self.linear_speed = []
        self.angular_speed = []

        # Set leader action
        self.linear_speed.append(1.0)
        self.angular_speed.append(0.0)

        # Set follower action

        for i in range(1, self.n):
            try:
                self.linear_speed.append(np.clip(action[i-1][0], -self.linear_speed_fixed_value, self.linear_speed_fixed_value))
                self.angular_speed.append(np.clip(action[i-1][1], -self.angular_speed_fixed_value, self.angular_speed_fixed_value))
            except:
                print(action)

        # We tell the Jetbot to move in this speed
        self.move_joints(self.linear_speed, self.angular_speed)

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        Find in The Doc you use
        :return:
        """
        jetbot_observations = []
        leader_yaw = 0

        for i in range(1, self.n):
            rel_yaw = []
            rel_dist = []
            rel_angle = []
            # We get the orientation of Jetbot in RPY
            roll, pitch, yaw = self.get_orientation_euler(i)
            if i == 0:
                leader_yaw = yaw
            rel_yaw.append(self.get_rel_yaw(yaw, leader_yaw))
            # We get the pos from the origin
            # get adjacent agent state
            for j in range(self.n):
                if j != i:
                    #if j == 0:
                        #print("follower", self.odom[i])
                        #print("leader", self.odom[j])
                    _rel_dist, _rel_angle = self.get_rel_pos(i, j)
                    rel_dist.append(np.min([10, _rel_dist]))
                    rel_angle.append(_rel_angle)

            # get lidar sscan
            lidar_obs = self.lidar_preprocess(i)
            jetbot_observations.append(np.array(rel_dist + rel_angle + rel_yaw + lidar_obs))

        # We get the current speed of the Jetbot
        '''
        we do not use the speed currently
        '''
        
        rospy.logdebug("Observations==>"+str(jetbot_observations))

        return jetbot_observations
        

    def _is_done(self, observations):
        return self.step_cnt > self.max_step

    def _compute_reward(self, observations, done):

        reward = [[0.0] for _ in range(1, self.n)]

        for i in range(1, self.n):
            # get relative position to leader
            dist2leader = observations[i-1][0]
            rospy.logdebug("dist2leader=" + str(dist2leader))
            ang2leader = observations[i-1][self.n-1]
            rospy.logdebug("ang2leader=" + str(ang2leader))
            yaw2leader = observations[i-1][2*(self.n-1)]
            rospy.logdebug("yaw2leader=" + str(yaw2leader))
            _, _, yaw = self.get_orientation_euler(i)
            angle_0 = self.wrap2pi(ang2leader + yaw2leader + np.pi)

            desired_pos = np.array([self.start_point.x[i] - self.start_point.x[0], self.start_point.y[i] - self.start_point.y[0]])
            current_pos = np.array([dist2leader*np.cos(angle_0), dist2leader*np.sin(angle_0)])

            err = np.linalg.norm(current_pos - desired_pos)

            lidar_left = observations[i-1][2 * (self.n - 1) + 2]
            lidar_right = observations[i-1][2 * (self.n - 1) + 4]

            avoidance = (1/min(lidar_left, 1.0) - 1/1.0) + (1/min(lidar_right, 1.0) - 1/1.0)

            reward[i-1][0] += - err - avoidance

            self.cumulated_reward += reward[i-1][0]
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))
        
        return reward


    # Private functions
    # Internal TaskEnv Methods
    # all for _get_obs
    def wrap2pi(self, angle):
        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < - np.pi:
            angle += 2 * np.pi
        return angle

    def get_orientation_euler(self, i):
        # We convert from quaternions to euler
        orientation_list = [self.odom[i].pose.pose.orientation.x,
                            self.odom[i].pose.pose.orientation.y,
                            self.odom[i].pose.pose.orientation.z,
                            self.odom[i].pose.pose.orientation.w]
    
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return roll, pitch, self.wrap2pi(self.start_yaw[i] + self.start_yaw_offset[i] + yaw)

    def get_orientation_euler_offset(self, i):
        # Since the odometry won't be reset, we calculate the offset value manually
        orientation_list = [self.odom[i].pose.pose.orientation.x,
                            self.odom[i].pose.pose.orientation.y,
                            self.odom[i].pose.pose.orientation.z,
                            self.odom[i].pose.pose.orientation.w]

        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return -roll, -pitch, -yaw

    def get_pos(self, i):
        return self.start_point.x[i] + self.start_point_offset.x[i] + self.odom[i].pose.pose.position.x, self.start_point.y[i] + self.start_point_offset.y[i] + self.odom[i].pose.pose.position.y, self.start_point.z[i] + self.start_point_offset.z[i] + self.odom[i].pose.pose.position.z

    def get_pos_offset(self, i):
        # Since the odometry won't be reset, we calculate the offset value manually
        return -self.odom[i].pose.pose.position.x, -self.odom[i].pose.pose.position.y, -self.odom[i].pose.pose.position.z

    def get_lidar_scan(self, i):
        self.scan_map[i] = {}
        angles = np.arange(self.lidar[i].angle_min, self.lidar[i].angle_max, self.lidar[i].angle_increment)
        ranges = list(self.lidar[i].ranges)
        for e, ang in enumerate(angles):
            if self.wrap2pi(ang) not in self.scan_map[i].keys():
                self.scan_map[i][self.wrap2pi(ang)] = np.clip(ranges[e], self.lidar[i].range_min, self.lidar[i].range_max)

    def get_rel_pos(self, i, j):
        x_i, y_i, _ = self.get_pos(i)
        x_j, y_j, _ = self.get_pos(j)
        x_diff = x_j - x_i
        y_diff = - (y_j - y_i)

        def rotate(x, y, theta):
            return x*np.cos(theta) - y*np.sin(theta), x*np.sin(theta) + y*np.cos(theta)
        rel_dist = np.linalg.norm([x_diff, y_diff])
        _, _, yaw = self.get_orientation_euler(i)

        x_diff_rotate, y_diff_rotate = rotate(x_diff, y_diff, yaw)
        rel_angle = np.arctan2(-y_diff_rotate, x_diff_rotate)

        return rel_dist, rel_angle

    def get_rel_yaw(self, yaw1, yaw2):
        return self.wrap2pi(yaw1 - yaw2)

    def lidar_preprocess(self, i):
        #res = dict(sorted(self.scan_map[i].items(), key=itemgetter(1))[:K])
        self.get_lidar_scan(i)
        lidar_item = list(self.scan_map[i].items())
        left_lidar = dict(lidar_item[:len(lidar_item)//2])
        right_lidar = dict(lidar_item[len(lidar_item)//2:-1])
        left_lidar_min = min(left_lidar, key=left_lidar.get)
        right_lidar_min = min(right_lidar, key=right_lidar.get)
        return [left_lidar_min, left_lidar[left_lidar_min], right_lidar_min, right_lidar[right_lidar_min]]