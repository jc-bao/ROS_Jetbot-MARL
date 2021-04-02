# Install

#### 1. Install [jetbot gazebo environment](https://github.com/issaiass/jetbot_diff_drive)

#### 2. Install [OpenAI for ROS](https://bitbucket.org/theconstructcore/openai_ros/src/kinetic-devel/)

```shell
cd ~/catkin_ws/src
git clone https://bitbucket.org/theconstructcore/openai_ros.git
cd ..
catkin_make
source devel/setup.bash
rosdep install openai_ros
# install OpenAI baselines
cd $THE_PATH_YOU_WANT_TO_INSTALL
git clone https://github.com/openai/baselines.git
cd baselines
pip install tensorflow==1.14 # pip install tensorflow-gpu==1.14
pip install -e .
```

#### 3. Install [Jetbot RL](https://github.com/jc-bao/jetbot_rl)

```shell
cd ~/catkin_ws/src
git clone https://github.com/jc-bao/jetbot_rl
cd ..
catkin_make
source devel/setup.bash
```

