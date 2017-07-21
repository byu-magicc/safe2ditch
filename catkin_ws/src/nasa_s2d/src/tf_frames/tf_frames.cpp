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

    // Translate and rotate into the body (base_link) frame -- remember that this is all in ENU instead of NED
    transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z + 6));

    tf::Quaternion quat(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

    if (true) {
        // Convert the attitude quaternion to Euler angles
        double phi, theta, psi;
        tf::Matrix3x3(quat).getRPY(phi, theta, psi);

        // Heading w.r.t ENU inertial frame (i.e., heading is zero at East)
        psi = M_PI;

        // Set the new attitude with specified heading
        quat.setRPY(phi, theta, psi);
    }
    
    transform.setRotation(quat);
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "fcu", "base_link")); // base_link is in ROS Body (REP 103) -- body NWU

}

// ----------------------------------------------------------------------------

}