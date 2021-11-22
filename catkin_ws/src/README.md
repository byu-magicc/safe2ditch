The following modifications should be run before catkin_make is run

dss
===
examples/vision_interface/ROSVisionInterface/ros_vision_interface.cpp

    if (itr.inlier_ratio >= 0.95) { ----> if (itr.model_likelihood >= 0.95) {

flat_earth_geolocation
======================
src/geolocator/geolocator.cpp

    track.inlier_ratio = msg->tracks[i].inlier_ratio; ----> track.model_likelihood = msg->tracks[i].model_likelihood;


visiona_opencv
==============
cv_bridge/CMakeLists.txt

    find_package(Boost REQUIRED python37) ----> find_package(Boost REQUIRED python3)
    add find_package(Boost REQUIRED python) after endif() at line 18, before set(_opencv_version 4)

vn200
=====
CMakeLists.txt

    find_package(catkin REQUIRED roscpp) ---> find_package(catkin REQUIRED roscpp tf2 tf2_geometry_msgs)
    CATKIN_DEPENDS roscpp sensor_msgs ---> CATKIN_DEPENDS roscpp sensor_msgs tf2 tf2_geometry_msgs
    include_directories(vnproglib-1.1.4.0/cpp/include ${catkin_INCLUDE_DIRS}) ---> include_directories(vnproglib-1.1.4.0/cpp/include ${catkin_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS})

