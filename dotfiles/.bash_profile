# Aliases
alias ll='ls -lh --color=auto'
alias l='ls -alh --color=auto'
alias ..='cd ..'
alias ...='cd ../..'

# ROS
source /opt/ros/kinetic/setup.bash

# ODROID ROS Commands
alias catkin_make_odroid='catkin_make -DCATKIN_WHITELIST_PACKAGES="nasa_s2d2;visual_mtt"'
