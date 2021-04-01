#!/usr/bin/env python
import os
import gym
from baselines import deepq
import rospy
import rospkg
# TODO[done] import your env
import jetbot_task_env

def main():
    rospy.init_node('jetbot_train_gym', anonymous=True, log_level=rospy.WARN)
    
    # Set the path where learned model will be saved
    # TODO[done] change the path to save
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('jetbot_rl_pkg')
    models_dir_path = os.path.join(pkg_path, "models_saved")
    if not os.path.exists(models_dir_path):
        os.makedirs(models_dir_path)
    
    out_model_file_path = os.path.join(models_dir_path, "jetbot_model.pkl") 
    
    
    max_timesteps = rospy.get_param("/jetbot_0/max_timesteps")
    buffer_size = rospy.get_param("/jetbot_0/buffer_size")
    # We convert to float becase if we are using Ye-X notation, it sometimes treats it like a string.
    lr = float(rospy.get_param("/jetbot_0/lr"))
    
    exploration_fraction = rospy.get_param("/jetbot_0/exploration_fraction")
    exploration_final_eps = rospy.get_param("/jetbot_0/exploration_final_eps")
    print_freq = rospy.get_param("/jetbot_0/print_freq")
    
    reward_task_learned = rospy.get_param("/jetbot_0/reward_task_learned")
    
    def callback(lcl, _glb):
        # stop training if reward exceeds 199
        aux1 = lcl['t'] > 100
        aux2 = sum(lcl['episode_rewards'][-101:-1]) / 100
        is_solved = aux1 and aux2 >= reward_task_learned
        
        rospy.logdebug("aux1="+str(aux1))
        rospy.logdebug("aux2="+str(aux2))
        rospy.logdebug("reward_task_learned="+str(reward_task_learned))
        rospy.logdebug("IS SOLVED?="+str(is_solved))
        
        return is_solved
    
    # TODO[done] change register's name
    env = gym.make("JetbotTaskEnv-v0")
    
    act = deepq.learn(
        env,
        network='mlp',
        lr=lr,
        total_timesteps=max_timesteps, 
        buffer_size=buffer_size,
        exploration_fraction=exploration_fraction,
        exploration_final_eps=exploration_final_eps,
        print_freq=print_freq, # how many apisodes until you print total rewards and info
        param_noise=False,
        callback=callback,
        load_path=out_model_file_path
    )
    
    env.close()


if __name__ == '__main__':
    main()