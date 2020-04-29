NASA S2D2 Emergency Landing Project
===================================

For information about this project and the hardware/software platform view the [wiki](https://magiccvs.byu.edu/gitlab/robust_tracking/safe2ditch/wikis/home).

This project contains git [submodules](http://bhilburn.org/cheat-sheet-for-git-submodules/). To clone this repo and all of the submodules:
```bash
$ git clone ssh://git@magiccvs.byu.edu:290/robust_tracking/safe2ditch.git # Make sure to add an SSH key to gitlab
$ cd safe2ditch && git checkout s2d_ros_standalone 
$ git submodule update --init --recursive
```

The submodule updates will require login information a github account that has
access to the NASA Safe2Ditch repository since it is a private repo.  If you
require access, contact Bryan Petty at bryan.j.petty@nasa.gov.

After installing the dependencies listed below, you can build the ROS packages:
```bash
$ cd catkin_ws
$ catkin_make
```

The catkin_make make command will take a few minutes to complete.  Once done,
you can test the build by running the software in the loop simulations.
Insturctions for this are below in the Software-in-the-Loop (SIL) section.

## Environment Setup ##

The easiest way to setup the Safe2Ditch environment is by sourcing `s2denv`. You can add it to your `~/.bashrc` for convenience, but beware: having multiple catkin workspaces sourced at once is generally a bad idea. This script is found in `safe2ditch/startup/s2devnv`. See [wiki/Software Setup](https://magiccvs.byu.edu/gitlab/robust_tracking/safe2ditch/wikis/software-setup) for other helpful environment setup tips.

### Software-in-the-Loop (SIL) ###

Make sure to setup the [`ardupilot_sim`](https://magiccvs.byu.edu/gitlab/lab/ardupilot_sim) package by following the instructions on its README. After you've installed all of its dependencies, make it run once (`roslaunch ardupilot_sim copter.launch`) so that it builds everything and you can verify that it is working (you may get `link down` errors, see the `ardupilot_sim` README for info).

After doing all that, run the Safe2Ditch SIL:

```bash
roslaunch nasa_s2d_sim sim.launch
```

Safe2Ditch can be activated when the copter is flying a mission in auto mode by
setting RC channel 6 above 1500.  This can be done by typing 
```bash
rc 6 rc_value
```
where rc_value is a number above 1500 and below 2000.  This command should be
typed into the mavproxy console connected to the simulation.

This uses [`gzsatellite`](https://github.com/plusk01/gzsatellite) to pull down satellite imagery and build a world. On your first run, you will have to wait ~5 minutes for it to download.

If you get `connection errors` in the `xterm` window that pops up, go into `nasa_s2d_sim/scripts/copter_sitl.sh` and increase the `sleep` time -- there is a note about slower computers.

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

One of the core pieces of the Safe2Ditch project is choosing viable *ditch sites* or landing zones. The Ditch Site Selector (DSS) component (`catkin_ws/src/dss`) is currently written in C++ and is actively being worked on by Bryan Petty, Analytical Mechanics Associates INC., under direction of Trish Glaab and Lou Glaab.

Contact Info: bryan.j.petty@nasa.gov

## Mission Planner ##

Because Mission Planner is built on Microsoft .NET, it is possible to run Mission Planner on Linux using the [mono project](http://www.mono-project.com/). Install `mono`, download Mission Planner, and then just run `sudo mono MissionPlanner.exe`. See [here](https://discuss.ardupilot.org/t/running-mission-planner-on-linux/19100).

In order for this to work, you must install `mono` from the official ppa and not from the one packaged with Ubuntu. Follow the instructions at the `mono` [project website](https://www.mono-project.com/download/stable/). Tested and works with Mission Planner 1.3.53 and `mono` 5.12.0.226 on Ubuntu 16.04 (and in Gnome).
