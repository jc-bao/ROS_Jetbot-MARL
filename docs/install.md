# Install

#### 1. Install [Jetbot gazebo environment](https://github.com/issaiass/jetbot_diff_drive)

Quick install instruction:

```
conda create -n ros ros-noetic-desktop -conda-forge -c robostack && conda activate ros

cd ~/catkin_ws/src
git clone --recursive https://github.com/jc-bao/Jetbot_RLFormationControl.git
cd ..; catkin_make
source devel/setup.bash
```

> Note: 
>
> If you want to use gazebo, remeber to install:
>
> `conda install -y -c robostack ros-noetic-gazebo-ros ros-noetic-gazebo-dev ros-noetic-gazebo-msgs ros-noetic-gazebo-plugins ros-noetic-gazebo-ros-control ros-noetic-gazebo-ros-pkgs ros-noetic-joint-state-controller ros-noetic-diff-drive-controller gazebo`

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

#### Extra: Install Mkdocs and [Materials](https://squidfunk.github.io/mkdocs-material/getting-started/)

```shell
conda install -y mkdocs mkdocs-material

cd $YOUR_WORK_DIRECTORY
mkdocs serve # check if the doc works @ http://localhost:8000/

mkdocs gh-deploy --force # push to github and deploy
```

