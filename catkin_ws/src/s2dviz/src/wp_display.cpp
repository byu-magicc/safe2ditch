#include "s2dviz/wp_display.h"

namespace s2dviz {

WPDisplay::WPDisplay()
  : Display()
{

  wp_topic_prop_.reset(new rviz::RosTopicProperty(
    "Waypoint Topic", "/mavros/mission/waypoints", 
    QString::fromStdString(ros::message_traits::datatype<mavros_msgs::WaypointList>()),
    "mavros waypoints", this, SLOT(updateTopic())));
}

// ----------------------------------------------------------------------------
// Protected Methods
// ----------------------------------------------------------------------------

void WPDisplay::onInitialize()
{
  Display::onInitialize();

}

// ----------------------------------------------------------------------------

void WPDisplay::reset()
{
  Display::reset();
  visuals_.clear();
}

// ----------------------------------------------------------------------------

void WPDisplay::onEnable()
{
  subscribe();
}

// ----------------------------------------------------------------------------

void WPDisplay::onDisable()
{
  unsubscribe();
  reset();
}

// ----------------------------------------------------------------------------
// Private Qt Slots
// ----------------------------------------------------------------------------

void WPDisplay::updateTopic()
{
  unsubscribe();
  reset();
  subscribe();
}

// ----------------------------------------------------------------------------
// Private Helper Methods
// ----------------------------------------------------------------------------

void WPDisplay::subscribe()
{
  // If the "enabled" checkbox is not true, bail
  if (!isEnabled()) return;

  // If the waypoint topic property is empty, bail
  if (wp_topic_prop_->getTopic().isEmpty()) return;

  try
  {
    sub_wp_ = update_nh_.subscribe(wp_topic_prop_->getTopicStd(), 1,
      &WPDisplay::processMessage, this);
  }
  catch (ros::Exception& e)
  {
    setStatus(rviz::StatusProperty::Error, "Topic",
      QString("Error subscribing: ") + e.what());
  }
  catch (...)
  {
    ROS_ERROR("Caught unknown exception!");
  }
}

// ----------------------------------------------------------------------------

void WPDisplay::unsubscribe()
{
  sub_wp_.shutdown();
}

// ----------------------------------------------------------------------------

void WPDisplay::processMessage(const mavros_msgs::WaypointListConstPtr& msg)
{

}

// ----------------------------------------------------------------------------

}

// Tell pluginlib about this class. It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(s2dviz::WPDisplay, rviz::Display)