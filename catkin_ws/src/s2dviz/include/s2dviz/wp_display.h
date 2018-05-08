#pragma once

#include <iostream>
#include <memory>

#include <QObject>

#include <geographic_msgs/GeoPoint.h>
#include <geodesy/utm.h>

#include <rviz/display.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/tf_frame_property.h>

#include <ros/ros.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/CommandCode.h>

#include <OgreMeshManager.h>

#include <rviz/display_context.h>

#include <rviz/default_plugin/markers/marker_base.h>
#include <rviz/default_plugin/markers/shape_marker.h>

#include <tf/transform_listener.h>
#include "rviz/frame_manager.h"

#include <visualization_msgs/Marker.h>

namespace s2dviz {

  class WPDisplay : public rviz::Display
  {
  Q_OBJECT
  public:
    WPDisplay();  

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
  protected:
    void onInitialize() override;

    // A helper to clear this display back to the initial state.
    void reset() override;

    void onEnable() override;
    void onDisable() override;

  private Q_SLOTS:
    void updateTopic();

  private:
    // ROS stuff
    ros::Subscriber sub_wp_;

    std::vector<rviz::MarkerBasePtr> markers_;

    // Method to handle an incoming ROS message.
    void processMessage(const mavros_msgs::WaypointListConstPtr& msg);

    // Helper functions
    void subscribe();
    void unsubscribe();

    // convert from LLA to UTM
    void convertLLAtoUTM(double lat, double lon, double& easting, double& northing);

    // User-editable property variables.
    std::unique_ptr<rviz::RosTopicProperty> wp_topic_prop_;  // mavros/WaypointList topic
    std::unique_ptr<rviz::RosTopicProperty> gwp_topic_prop_; // mavros/Waypoint topic
    std::unique_ptr<rviz::TfFrameProperty> tf_frame_prop_;   // TF frame for UTM origin

  };

}