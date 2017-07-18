#include "tf_frames/tf_frames.h"

namespace tf_frames {

TFFrames::TFFrames() :
    nh_(ros::NodeHandle())
{
    // Set up Publishers and Subscribers
    sub_uav_ = nh_.subscribe("uav_pose", 1, &TFFrames::cb_uav, this);
}

// ----------------------------------------------------------------------------

void TFFrames::cb_uav(const geometry_msgs::PoseStampedPtr& msg)
{
    // Create an empty transform
    tf::Transform transform;

    // Link the camera to the quad body
    transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    transform.setRotation(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "fcu", "fcu_body")); // fcu_body is in ROS Body (REP 103) -- body NWU

}

// ----------------------------------------------------------------------------

}