Safe2Ditch Monte Carlo Simulations
==================================

This package contains support code for running Safe2Ditch Monte Carlo simulations.

## Setting Up MC Sims ##

First, you must make sure that you have all of the setup/versions the same on all of the machines you would like to run the simulations.

Monte Carlo sims are ran when `mcsim.py` calls `roslaunch montecarlo sim.launch viz:=false`. This underyling launch file is also useful when directly called for visualizing the area (taking screnshots for plots/images).