#include "s2dviz/wp_display.h"

namespace s2dviz {

WPDisplay::WPDisplay()
  : Display()
{

  // Property: Waypoint topic
  wp_topic_prop_.reset(new rviz::RosTopicProperty(
    "Waypoint Topic", "/mavros/mission/waypoints", 
    QString::fromStdString(ros::message_traits::datatype<mavros_msgs::WaypointList>()),
    "mavros mission waypoints", this, SLOT(updateTopic())));

  // Property: Guided waypoint topic
  gwp_topic_prop_.reset(new rviz::RosTopicProperty(
    "Guided Waypoint Topic", "", 
    QString::fromStdString(ros::message_traits::datatype<mavros_msgs::Waypoint>()),
    "current guided waypoint", this, SLOT(updateTopic())));

  // Property: TF Frame of the UTM origin
  tf_frame_prop_.reset(new rviz::TfFrameProperty(
    "Local UTM Origin Frame", "utm_origin",
    "TF frame for the local UTM origin wrt the base_link.",
    this, nullptr, false, nullptr, nullptr));
}

// ----------------------------------------------------------------------------
// Protected Methods
// ----------------------------------------------------------------------------

void WPDisplay::onInitialize()
{
  Display::onInitialize();
  tf_frame_prop_->setFrameManager(context_->getFrameManager());
}

// ----------------------------------------------------------------------------

void WPDisplay::reset()
{
  Display::reset();
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

  // Clear out the store of shape markers
  markers_.clear();

  for (int i=0; i<msg->waypoints.size(); i++)
  {
    // for convenience
    auto wp = msg->waypoints[i];

    // we only care about waypoint mission items
    if (wp.command != mavros_msgs::CommandCode::NAV_WAYPOINT) continue;

    // Create a new rviz::ShapeMarker and keep track of it
    rviz::MarkerBasePtr marker;
    marker.reset(new rviz::ShapeMarker(nullptr, context_, scene_node_));
    markers_.push_back(marker);

    // convert waypoint LLA to UTM
    double easting = 0, northing = 0;
    convertLLAtoUTM(wp.x_lat, wp.y_long, easting, northing);


    // Create the visualization message which controls the marker appearance
    visualization_msgs::Marker marker_msg;
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.header.frame_id = "utm_origin";
    marker_msg.type = visualization_msgs::Marker::SPHERE;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.id = i;
    marker_msg.pose.position.x = easting;
    marker_msg.pose.position.y = northing;
    marker_msg.pose.position.z = wp.z_alt;
    marker_msg.pose.orientation.w = 1;
    marker_msg.scale.x = 1;
    marker_msg.scale.y = 1;
    marker_msg.scale.z = 1;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;
    marker_msg.frame_locked = true;

    marker->setMessage(marker_msg);
  }

  // context_->queueRender();
}

// ----------------------------------------------------------------------------

void WPDisplay::convertLLAtoUTM(double lat, double lon, double& easting, double& northing)
{
  // Convert lat/lon to UTM
  // http://answers.ros.org/question/50763/need-help-converting-lat-long-coordinates-into-meters/?answer=257140#post-id-257140
  geographic_msgs::GeoPoint geo_pt;
  geo_pt.latitude = lat;
  geo_pt.longitude = lon;
  geodesy::UTMPoint utm_pt(geo_pt);

  easting = utm_pt.easting;
  northing = utm_pt.northing;
}

// ----------------------------------------------------------------------------

}

// Tell pluginlib about this class. It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(s2dviz::WPDisplay, rviz::Display)