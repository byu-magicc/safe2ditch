Safe2Ditch Startup Files
========================

A collection of shell scripts and aliases to setup the Safe2Ditch environment.

## `s2denv` ##

Setup the Safe2Ditch environment for use with ROS, etc. This file can be safely source by non-flight computers (i.e., your working desktop computer).

In addition to sourcing the `safe2ditch/catkin_ws` workspace, it creates a series of `S2D*` environment variable sthat are used for data logging and other flight / sim configuration.

Also, this script exposes the following functions:

- `setup_s2denv AIRCRAFT MASTER_URI MYIP`
	- the following defaults are used: `MASTER_URI=http://localhost:11311`, `ROS_IP=127.0.0.1`, and `BAGPATH=$HOME`
	- this needs to be run before your first test flight, or if you ever change the ROS network
- `testname NAME`
	- Sets a flight test name. This is useful for keeping data organized and related to test flights.
	- For example, before running a vision test, you could run `testname vision_1obj` and all ROS bags and log files will be prepended with that.
- `s2denv`
	- Prints environment variables for S2D and ROS.


## `tx2rc` ##

Environment setup on a TX2. Place the following at the bottom of `~/.bashrc`:

```bash
# Setup NVIDIA TX2 Safe2Ditch environment
source ~/dev/safe2ditch/startup/tx2rc
```

This will automatically source and setup the `s2denv` bash script

## `rosboot.sh` ##

Used by `safe2ditch.service` to setup the system and run the code / bag recorder on boot.

## `safe2ditch.service` ##

A `systemd` service file to control the automatic start/stop of Safe2Ditch ROS system on bootup (very useful for flight testing).

Install with:

```bash
$ sudo ln -s ~/dev/safe2ditch/startup/safe2ditch.service /etc/systemd/system/safe2ditch.service
```

Then you can use `systemctl` commands to `enable` / `disable` Safe2Ditch from starting on boot. To manually turn on and off, use the following commands

```bash
$ sudo systemctl enable safe2ditch.service  # installs for boot
$ sudo systemctl disable safe2ditch.service # disables autostart on boot
# ----
$ sudo systemctl start safe2ditch.service  # kicks off the roslaunch
$ sudo systemctl stop safe2ditch.service   # stops ROS and bag recording
```

Note that this service file is calling the `rosboot.sh` script, which starts a bagrecord.

This functionality is meant to be used in the field so that when the on-board computer is booted, Safe2Ditch ROS and a ROS bag recorder (named using `testname`, see `s2denv`) start. In order to cleanly shutdown the bag recorder, it is useful if you can manually stop the service after a flight, before powering down the computer. Otherwise, you will see a lot of `*.bag.active` files, which you will have to `reindex` -- which typically isn't a problem, but you risk data loss.
