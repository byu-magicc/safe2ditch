#include "hud/hud.h"

namespace hud {

HUD::HUD()
{
    // ROS stuff
    image_transport::ImageTransport it(nh_);
    pub_hud_  = it.advertise("hud/image_raw", 1);
    sub_cam_  = it.subscribe("tracks_video", 1, &HUD::cb_cam, this);
    sub_odom_ = nh_.subscribe("odom", 1, &HUD::cb_odom, this);
    sub_state_ = nh_.subscribe("mavros/state", 1, &HUD::cb_state, this);
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void HUD::cb_cam(const sensor_msgs::ImageConstPtr& frame)
{
    // Convert from sensor_msg to cv::Mat
    cv::Mat img = cv_bridge::toCvCopy(frame, "bgr8")->image;

    // Write mode
    std::string mode_text = "Mode: " + last_state_msg_.mode;
    cv::Point corner = cv::Point(20, img.rows - 60);
    cv::putText(img, mode_text, corner, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

    // Write altitude
    char text[40];
    sprintf(text, "Altitude: %.0f m", last_odom_msg_.pose.pose.position.z);
    corner = cv::Point(20, img.rows - 20);
    cv::putText(img, text, corner, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

    // Convert back to sensor_msg and publish
    pub_hud_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg());
}

// ----------------------------------------------------------------------------

void HUD::cb_odom(const nav_msgs::Odometry& msg)
{
    last_odom_msg_ = msg;
}

// ----------------------------------------------------------------------------

void HUD::cb_state(const mavros_msgs::State& msg)
{
    last_state_msg_ = msg;
}

}