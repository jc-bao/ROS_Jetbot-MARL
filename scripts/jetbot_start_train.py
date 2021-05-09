#!/usr/bin/env python
import os
import gym
import rospy
import rospkg
from tf2rl.algos.ddpg import DDPG
from tf2rl.experiments.trainer import Trainer
# TODO[done] import your env
import jetbot_formation_env

def main():
    rospy.init_node('jetbot_train_gym', anonymous=True, log_level=rospy.WARN)
    
    # Set the path where learned model will be saved
    # TODO[done] change the path to save
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('jetbot_rl_pkg')
    models_dir_path = pkg_path
    save_path = os.path.join("/mnt/ssd_raid0/home/qiuxy/catkin_ws/src/jetbot_rl", "models_saved")
    #out_model_file_path = os.path.join(models_dir_path, "jetbot_model.pkl")
    
    
    max_timesteps = rospy.get_param("/max_timesteps")
    buffer_size = rospy.get_param("/buffer_size")
    # We convert to float becase if we are using Ye-X notation, it sometimes treats it like a string.
    lr = float(rospy.get_param("/lr"))
    
    exploration_fraction = rospy.get_param("/exploration_fraction")
    exploration_final_eps = rospy.get_param("/exploration_final_eps")
    print_freq = rospy.get_param("/print_freq")
    
    reward_task_learned = rospy.get_param("/reward_task_learned")
    
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
    env = gym.make("JetbotFormationEnv-v0")

    parser = Trainer.get_argument()
    parser = DDPG.get_argument(parser)
    #parser.set_defaults(model_dir=models_dir_path)
    parser.set_defaults(logdir=save_path)
    parser.set_defaults(test_interval=2000)
    parser.set_defaults(max_steps=2500000)
    parser.set_defaults(gpu=-1)
    parser.set_defaults(n_warmup=0)
    parser.set_defaults(batch_size=32)
    parser.set_defaults(memory_capacity=int(1e4))
    parser.add_argument('--env-name', type=str, default="JetbotFormationEnv-v0")
    #args = parser.parse_args()
    args, unknown = parser.parse_known_args()

    policy = DDPG(
        state_shape=env.observation_space.shape,
        action_dim=env.action_space.low.size,
        discount=0.99,
        gpu=args.gpu,
        memory_capacity=args.memory_capacity,
        batch_size=args.batch_size,
        n_warmup=args.n_warmup,
        update_interval=args.update_interval)

    trainer = Trainer(policy, env, args)
    trainer()

    #env.close()


if __name__ == '__main__':
    main()