#pragma once

#include <ros/ros.h>
#include <tf/tf.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace tf_frames {

    class TFFrames
    {
    public:
        TFFrames();

    private:
        // Node handles, publishers, subscribers
        ros::NodeHandle nh_;
        ros::Subscriber sub_uav_;

        // ROS tf listener and broadcaster
        tf::TransformListener tf_listener_;
        tf::TransformBroadcaster tf_br_;

        // Functions
        void cb_uav(const geometry_msgs::PoseStampedPtr& msg);
        
    };

}