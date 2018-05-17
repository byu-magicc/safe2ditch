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

For now, [ueye_cam](https://github.com/anqixu/ueye_cam) does not have a Jenkins job for `armhf` in the ROS buildfarm. Until that gets resolved, I have added `ueye_cam` as a git submodule. This package depends on having the IDS Software Suite and SDK installed on this machine, see [here](https://magiccvs.byu.edu/gitlab/robust_tracking/nasa-s2d2/wikis/software-setup).
**Update:** You can now install armhf version of `ueye_cam` through `sudo apt install ros-kinetic-ueye-cam`


## Launch File Structure ##

There are three main launch files to work with in Safe2Ditch: `hardware.launch`, `sim.launch`, and `review.launch`. The utility of each of these launch top-level launch files are shown pictorially below.

<p style="text-align: center">
  <img src="https://magiccvs.byu.edu/gitlab/robust_tracking/safe2ditch/wikis/assets/s2d_roslaunch.svg" width="80%" />
</p>

## Consistent Hardware/Simulation Communications ##

To achieve a consistent interface across simulation and hardware testing of Safe2Ditch, the following communication architecture is followed.

<p style="text-align: center">
  <img src="https://magiccvs.byu.edu/gitlab/robust_tracking/safe2ditch/wikis/assets/s2d_comms.svg" width="50%" />
</p>

## Ditch Site Selector (DSS) ##

One of the core pieces of the Safe2Ditch project is choosing viable *ditch sites* or landing zones. The Ditch Site Selector (DSS) component (`catkin_ws/src/dss/src`) is currently written in Python and is actively being worked on by Trish Glaab.

## Mission Planner ##

Because Mission Planner is built on Microsoft .NET, it is possible to run Mission Planner on Linux using the [mono project](http://www.mono-project.com/). Install `mono`, download Mission Planner, and then just run `sudo mono MissionPlanner.exe`. See [here](https://discuss.ardupilot.org/t/running-mission-planner-on-linux/19100).

In order for this to work, you must install `mono` from the official ppa and not from the one packaged with Ubuntu. Follow the instructions at the `mono` [project website](https://www.mono-project.com/download/stable/). Tested and works with Mission Planner 1.3.53 and `mono` 5.12.0.226 on Ubuntu 16.04 (and in Gnome).