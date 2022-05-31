NASA S2D2 Emergency Landing Project
===================================

For information about this project and the hardware/software platform view the [wiki](https://magiccvs.byu.edu/gitlab/robust_tracking/safe2ditch/wikis/home).

WARNING: This implementation assumes that you will be running the STEReO branch of 
ICAROUS being that is being developed by NASA in parallel to Safe2Ditch.  This
branch has restricted access, so use of the icarous_s2d branch should be for
advanced users with the appropriate access rights only.

This project contains git [submodules](http://bhilburn.org/cheat-sheet-for-git-submodules/). To clone this repo and all of the submodules:
```bash
$ git clone ssh://git@magiccvs.byu.edu:290/robust_tracking/safe2ditch.git # Make sure to add an SSH key to gitlab
$ cd safe2ditch && git checkout icarous_vn200_xavier
$ git submodule update --init --recursive
```

The submodule updates will require login information a github account that has
access to the NASA Safe2Ditch repository since it is a private repo.  If you
require access, contact Bryan Petty at bryan.j.petty@nasa.gov.

After installing the dependencies listed below, you can build the ROS packages:
```bash
$ cd catkin_ws
$ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so
```

The catkin_make make command will take a few minutes to complete.

## Environment Setup ##

The easiest way to setup the Safe2Ditch environment is by sourcing `s2denv`. You can add it to your `~/.bashrc` for convenience, but beware: having multiple catkin workspaces sourced at once is generally a bad idea. This script is found in `safe2ditch/startup/s2devnv`. See [wiki/Software Setup](https://magiccvs.byu.edu/gitlab/robust_tracking/safe2ditch/wikis/software-setup) for other helpful environment setup tips.

## Dependencies ##

- Ubuntu 18.04.1 LTS (Bionic Beaver)
- ROS Melodic
- See Compilation and Execution on Nvidia Jetson Xavier below for more dependencies and install instructions
- `sudo apt-get install ros-kinetic-joy ros-kinetic-serial libncurses5-dev`

For now, [ueye_cam](https://github.com/anqixu/ueye_cam) does not have a Jenkins job for `armhf` in the ROS buildfarm. Until that gets resolved, I have added `ueye_cam` as a git submodule. This package depends on having the IDS Software Suite and SDK installed on this machine, see [here](https://magiccvs.byu.edu/gitlab/robust_tracking/nasa-s2d2/wikis/software-setup).
**Update:** You can now install armhf version of `ueye_cam` through `sudo apt install ros-kinetic-ueye-cam`


## Launch File Structure ##

NOTE: On this branch, only `hardware.launch` is supported.

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

One of the core pieces of the Safe2Ditch project is choosing viable *ditch sites* or landing zones. The Ditch Site Selector (DSS) component (`catkin_ws/src/dss`) is currently written in C++ and is actively being worked on by Bryan Petty, under direction of Trish Glaab and Lou Glaab.

Contact Info: bryan.j.petty@nasa.gov

## Mission Planner ##

Because Mission Planner is built on Microsoft .NET, it is possible to run Mission Planner on Linux using the [mono project](http://www.mono-project.com/). Install `mono`, download Mission Planner, and then just run `sudo mono MissionPlanner.exe`. See [here](https://discuss.ardupilot.org/t/running-mission-planner-on-linux/19100).

In order for this to work, you must install `mono` from the official ppa and not from the one packaged with Ubuntu. Follow the instructions at the `mono` [project website](https://www.mono-project.com/download/stable/). Tested and works with Mission Planner 1.3.53 and `mono` 5.12.0.226 on Ubuntu 16.04 (and in Gnome).

## Invlaid Operation in VN200 Module ##
Generally this happens due to permissions on the usb serial port.  To solve this, type this in the command line
'sudo chmod a+rw /dev/ttyUSB0'

## Compilation and Execution on Nvidia Jetson Xavier ##
- Install ROS Base: http://wiki.ros.org/melodic/Installation/Ubuntu
- uninstall ros tf and tf2 if installed: sudo apt-get remove ros-melodic-tf ros-melodic-tf2
- Pull submodules (see above)
- Read README.md in both required_installs and catkin_ws/src
- set python3 as default: sudo unlink /usr/bin/python && sudo ln -s /usr/bin/python3.6 /usr/bin/python
- install python3 tools: sudo apt-get install -y python3-pip python3-dev python3-setuptools
- pip3 install catkin_pkg
- sudo apt-get install ros-melodic-mavlink ros-melodic-uuid-msgs ros-melodic-tf2-bullet python3-numpy libgeographic-dev ros-melodic-camera-calibration ros-melodic-unique-id ros-melodic-camera-info-manager apt-get install python3-empy geographiclib-tools python3-future python3-defusedxml python3-netifaces python3-pycryptodome python3-gnupg python3-pyproj
- sudo ./catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
- install ueye drivers: https://www.ids-imaging.us/downloads.html (version 4.91.1.0_arm64 used)
- compile with catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_BLACKLIST_PACKAGES="test_mavros"

For issue with Installation or execution, contact Bryan J Petty at bryan.j.petty@nasa.gov.
