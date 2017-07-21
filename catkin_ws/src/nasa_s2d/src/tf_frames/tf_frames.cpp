#include "tf_frames/tf_frames.h"

namespace tf_frames {

TFFrames::TFFrames() :
    nh_(ros::NodeHandle())
{
    ros::NodeHandle nh_private("~");

    // Set up Publishers and Subscribers
    sub_uav_ = nh_.subscribe("uav_pose", 1, &TFFrames::cb_uav, this);

    // ROS Services
    srv_uavpose_ = nh_private.advertiseService("set_uav_pose", &TFFrames::srv_set_pose, this);
}

// ----------------------------------------------------------------------------

void TFFrames::cb_uav(const geometry_msgs::PoseStampedPtr& msg)
{
    // Create an empty transform
    tf::Transform transform;

    // Put current pose into tf data structures
    tf::Vector3 origin(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    tf::Quaternion quat(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

    // Allow the user to override the heading and add a position offset to the tf
    if (pose_override_) {

        //
        // Heading
        //

        // Convert the attitude quaternion to Euler angles
        double phi, theta, psi;
        tf::Matrix3x3(quat).getRPY(phi, theta, psi);

        // Heading w.r.t ENU inertial frame (i.e., heading is zero at East)
        if (std::abs(heading_) < 2*M_PI)
            psi = heading_;

        // Set the new attitude with specified heading
        quat.setRPY(phi, theta, psi);

        //
        // Position Offset
        //

        // Origin is w.r.t ENU inertial frame
        origin += position_offset_;
    }

    // Translate and rotate into the body (base_link) frame -- remember that this is all in ENU instead of NED
    transform.setOrigin(origin);
    transform.setRotation(quat);
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "fcu", "base_link")); // base_link is in ROS Body (REP 103) -- body NWU

}

// ----------------------------------------------------------------------------

bool TFFrames::srv_set_pose(nasa_s2d::SetUAVPose::Request &req, nasa_s2d::SetUAVPose::Response &res)
{
    pose_override_ = true;

    // Allow the user to set the UAV heading
    // Heading w.r.t ENU inertial frame (i.e., heading is zero at East)
    heading_ = req.heading_enu*M_PI/180;

    // Provide an offset to the position of the UAV
    // Position is w.r.t ENU inertial frame
    position_offset_ = tf::Vector3(req.offset_enu.x, req.offset_enu.y, req.offset_enu.z);
}

}