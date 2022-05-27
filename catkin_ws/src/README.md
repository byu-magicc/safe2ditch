The following modifications should be run before catkin_make is run

flat_earth_geolocation
======================
src/geolocator/geolocator.cpp

    track.inlier_ratio = msg->tracks[i].inlier_ratio; ----> track.model_likelihood = msg->tracks[i].model_likelihood;


visiona_opencv
==============
cv_bridge/CMakeLists.txt

    find_package(Boost REQUIRED python37) ----> find_package(Boost REQUIRED python3)

vn200
=====
CMakeLists.txt

    find_package(catkin REQUIRED roscpp) ---> find_package(catkin REQUIRED roscpp tf2 tf2_geometry_msgs)
    CATKIN_DEPENDS roscpp sensor_msgs ---> CATKIN_DEPENDS roscpp sensor_msgs tf2 tf2_geometry_msgs
    include_directories(vnproglib-1.1.4.0/cpp/include ${catkin_INCLUDE_DIRS}) ---> include_directories(vnproglib-1.1.4.0/cpp/include ${catkin_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS})

Compile
=======
run catkin_make in the following way. This tells catkin to use python3 for compilation instead of python2.

    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_BLACKLIST_PACKAGES="test_mavros"
