#pragma once

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/State.h"

namespace hud {

    class HUD
    {
    public:
        HUD();

    private:
        // ROS
        ros::NodeHandle nh_;
        image_transport::Subscriber sub_cam_;
        image_transport::Publisher pub_hud_;
        ros::Subscriber sub_odom_, sub_state_;

        // latest messages
        nav_msgs::Odometry last_odom_msg_;
        mavros_msgs::State last_state_msg_;

        // ROS callbacks
        void cb_cam(const sensor_msgs::ImageConstPtr& frame);
        void cb_odom(const nav_msgs::Odometry& msg);
        void cb_state(const mavros_msgs::State& msg);
    };

}