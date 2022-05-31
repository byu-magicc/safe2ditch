The following modifications should be run before catkin_make is run

flat_earth_geolocation
======================
src/geolocator/geolocator.cpp

    track.inlier_ratio = msg->tracks[i].inlier_ratio; ----> track.model_likelihood = msg->tracks[i].model_likelihood

Compile
=======
run catkin_make in the following way. This tells catkin to use python3 for compilation instead of python2.

    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so -DCATKIN_BLACKLIST_PACKAGES="test_mavros"
