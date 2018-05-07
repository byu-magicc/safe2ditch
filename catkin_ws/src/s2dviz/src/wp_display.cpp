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
  // subscribe();

  // markers_.push_back(createMarker(message->type, this, context_, scene_node_));
  rviz::MarkerBasePtr marker;
  marker.reset(new rviz::ShapeMarker(nullptr, context_, scene_node_));
  markers_.push_back(marker);


  visualization_msgs::Marker msg;

  msg.type = visualization_msgs::Marker::SPHERE;
  msg.action = visualization_msgs::Marker::ADD;
  msg.id = 1;
  msg.pose.position.x = 5;
  msg.pose.position.y = 1;
  msg.pose.position.z = 1;
  msg.pose.orientation.w = 1;
  msg.scale.x = 10;
  msg.scale.y = 10;
  msg.scale.z = 10;
  msg.color.r = 1.0;
  msg.color.g = 1.0;
  msg.color.b = 1.0;
  msg.color.a = 1.0;


  marker->setMessage(msg);
  marker->setMessage(msg);

  context_->queueRender();

  ROS_ERROR("sent a messag");
  setStatus(rviz::StatusProperty::Ok, "Message", QString("sent"));
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