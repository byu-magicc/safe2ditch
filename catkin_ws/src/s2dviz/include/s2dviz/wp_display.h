#pragma once

#include <iostream>
#include <memory>

#include <QObject>

#include <rviz/display.h>
#include <rviz/properties/ros_topic_property.h>

#include <ros/ros.h>
#include <mavros_msgs/WaypointList.h>

#include "s2dviz/wp_visual.h"

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

    // collection of waypoint visuals
    std::vector<std::unique_ptr<WPVisual>> visuals_;

    // Method to handle an incoming ROS message.
    void processMessage(const mavros_msgs::WaypointListConstPtr& msg);

    // Helper functions
    void subscribe();
    void unsubscribe();

    // User-editable property variables.
    std::unique_ptr<rviz::RosTopicProperty> wp_topic_prop_;

  };

}