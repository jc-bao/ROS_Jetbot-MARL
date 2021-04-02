# Reinforcement Learning for Jetbot

## Introduction

This project aimed to build an applicable RL training environment for jetbot.

## Project layout

    package.xml    # The package configuration file.
    CMakeList.txt  # The Cmake file
    launch/
    		start_training.launch # launch file used in python
    configre/
    		jetbot_params_deepQ # set parameter for training
    scripts/
    		jetbot_start_deepq.py # training entry point
    		jetbot_task_env.py    # task environment script
    		jetbot_robot_env.pt   # robot environment script
    docs/
        index.md  # The documentation homepage.
        ...       # Other markdown pages, images and other files.

