# Shore-Middleware

This repository hosts modified ROS1 middleware for __Shore__.

Most of the modifications are in the `ros_comm/roscpp/` folder. These modifications primarily involve:
1. Integrating additional logic into the vanilla scheduler.
2. Passing shoreline information to associate it with user-level tasks and kernel tasks."

This installation guided is tested on **ubuntu 20.04**.


# Installation Guide

## Prerequisite

Install the dependency packages
```
$ sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstools python3-vcstool build-essential
```

Initializing ROS dep
```
$ sudo rosdep init
$ rosdep update
```


## Compilation

Create a directory for the ROS workspace
```
$ mkdir -p ~/ros_catkin_ws
$ cd ~/ros_catkin_ws
```

Clone the project and name it as `src/`. This is the naming convention of catkin workspace 
```
$ git clone https://github.com/WUSTL-CSPL/Shore-Middleware src
```

Install all dependencies via `rosdep`
```
$ cd ~/ros_catkin_ws
$ rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y
```


Compiling the project
```
$ cd ~/ros_catkin_ws
$ ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF
```

To include ROS in your environment path
```
$ source ~/ros_catkin_ws/install_isolated/setup.bash
```

[optional] To make this automatic, put it into ~/.bashrc
```
$ echo "source ~/ros_catkin_ws/install_isolated/setup.bash" >> ~/.bashrc
```

##  Turn on/off Shore-related code


Turn on/off Shore-related scheduling
```
// In ros_comm/roscpp/include/ros/param.h
#define Shore_Sched
```

Turn on/off debugging logs
```
// In ros_comm/roscpp/include/ros/param.h
// #define Shore_Debug
```

Then recompile the project.
```
$ cd ~/ros_catkin_ws
$ ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
```