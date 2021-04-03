# Common Issues

#### Gazebo Issues

###### 1. PID reading error

```
[ERROR] [1617369469.665635903, 0.160000000]: No p gain specified for pid.  Namespace: /jetbot_0/gazebo_ros_control/pid_gains/lt_wheel_joint
[ERROR] [1617369469.666604764, 0.160000000]: No p gain specified for pid.  Namespace: /jetbot_0/gazebo_ros_control/pid_gains/rt_wheel_joint
```
> [wait for solution]

###### 2. Jetbot not moving in Gazebo during the training 

![](https://tva1.sinaimg.cn/large/008eGmZEly1gp5pykavqij31sk0fawmh.jpg)

> [wait for solution]

#### Training Issues

###### 1. Training failed after change the parameter

```
2021-04-02 13:58:20.484572: I tensorflow/core/platform/profile_utils/cpu_utils.cc:112] CPU Frequency: 2900010000 Hz
/usr/local/lib/python3.8/dist-packages/numpy/core/fromnumeric.py:3372: RuntimeWarning: Mean of empty slice.
  return _methods._mean(a, axis=axis, dtype=dtype,
/usr/local/lib/python3.8/dist-packages/numpy/core/_methods.py:170: RuntimeWarning: invalid value encountered in double_scalars
  ret = ret.dtype.type(ret / rcount)
```

> Solution:
>
> 1. Close Gazebo and other rqt windows
> 2. Delete `saved_model`

#### Server Issues

###### 1. Gazebo come up with a problem while rqt runs fine.

```
libGL error: No matching fbConfigs or visuals found
libGL error: failed to load driver: swrast
```

> [wait for solution]