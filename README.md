NASA S2D2 Emergency Landing Project
===================================

For information about this project and the hardware/software platform view the [wiki](https://magiccvs.byu.edu/gitlab/robust_tracking/nasa-s2d2/wikis/home).

This project contains git [submodules](http://bhilburn.org/cheat-sheet-for-git-submodules/). To clone this repo and all of the submodules:
```bash
$ git clone ssh://git@magiccvs.byu.edu:290/robust_tracking/nasa-s2d2.git # Make sure to add an SSH key to gitlab
$ git submodule update --init --recursive
```

After installing the dependencies listed below, you can build the ROS packages:
```bash
$ cd catkin_ws
$ catkin_make
```

## Dependencies ##

- Ubuntu 16.04.1 LTS (Xenial)
- ROS Kinetic
- `sudo apt-get install ros-kinetic-joy ros-kinetic-serial libncurses5-dev`