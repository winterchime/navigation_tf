# navigation_tf

## Getting Started

```bash
git clone git@github.com:winterchime/navigation_tf.git
```

```bash
mv navigation_tf/ your_ws
```

```bash
cd your_ws
```

```bash
catkin_make
```

In Shell 1
```bash
roscore
```

In Shell 2
```bash
rosrun robot_setup_tf tf_broadcaster
```

In Shell 3
```bash
rosrun robot_setup_tf tf_listener
```

[More Information](http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF)
