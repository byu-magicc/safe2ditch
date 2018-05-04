#pragma once

// NOTE: workaround for issue: https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#endif

#include <memory>

#include <ros/ros.h>

#include <rviz/display.h>
#include <rviz/properties/ros_topic_property.h>

#include <QObject>

namespace s2dviz {

  class WPDisplay : public rviz::Display
  {
  Q_OBJECT
  public:
    WPDisplay();
    ~WPDisplay();
  

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
  protected:
    void onInitialize() override;

    // A helper to clear this display back to the initial state.
    void reset() override;


  private:
    // Function to handle an incoming ROS message.
 

    // User-editable property variables.
    std::unique_ptr<rviz::RosTopicProperty> wp_topic_prop_;

  };

}