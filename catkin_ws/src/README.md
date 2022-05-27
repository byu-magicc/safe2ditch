The following modifications should be run before catkin_make is run

flat_earth_geolocation
======================
src/geolocator/geolocator.cpp

    track.inlier_ratio = msg->tracks[i].inlier_ratio; ----> track.model_likelihood = msg->tracks[i].model_likelihood;


visiona_opencv
==============
cv_bridge/CMakeLists.txt

    find_package(Boost REQUIRED python37) ----> find_package(Boost REQUIRED python3)

Compile
=======
run catkin_make in the following way. This tells catkin to use python3 for compilation instead of python2.

    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_BLACKLIST_PACKAGES="test_mavros"
