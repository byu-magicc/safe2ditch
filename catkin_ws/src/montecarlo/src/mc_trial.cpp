/**
 * @file mc_trial.cpp
 * @author Parker Lusk <parkerclusk@gmail.com>
 */

#include "montecarlo/mc_trial.h"

#include <cmath>

#include <geographic_msgs/GeoPoint.h>

namespace montecarlo {

MCTrial::MCTrial(int Nt, int m)
: Nt_(Nt), m_(m)
{
  targets_.resize(Nt);
  targets_out_of_view_.resize(Nt);
}

// ----------------------------------------------------------------------------

void MCTrial::recv_msg_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  msg_pose_ = msg;
}

// ----------------------------------------------------------------------------

void MCTrial::recv_msg_state(const mavros_msgs::State::ConstPtr& msg)
{
  msg_state_ = msg;
}

// ----------------------------------------------------------------------------
  
void MCTrial::recv_msg_ditchsites(const nasa_s2d::DitchSiteList::ConstPtr& msg)
{
  msg_ds_ = msg;

  for (auto&& ds : msg_ds_->ditch_sites)
  {
    if (ds.selected)
    {
      // If the previously selected ditch site is empty then we are now engaged
      if (!safe2ditch_engaged_ && current_ds_.name.empty()) safe2ditch_engaged_ = true;

      current_ds_ = ds;
    }
  }
}

// ----------------------------------------------------------------------------

void MCTrial::recv_msg_home(const mavros_msgs::HomePosition::ConstPtr& msg)
{
  msg_home_ = msg;
}

// ----------------------------------------------------------------------------

void MCTrial::recv_msg_target(int n, const nav_msgs::Odometry::ConstPtr& msg)
{
  // because we don't have access to the coordinates of the geolocated camera
  // frustum, just do this hack. I looked at a simulation and the ditch site
  // is pretty much inscribed in the frustum when the multirotor is 15 m AGL.
  if (msg_pose_ != nullptr && safe2ditch_engaged_ && msg_pose_->pose.position.z >= 15.0)
  {
    // this is the position of the ith target before the camera FOV was smaller
    // than the ditch site. If the target was not in the ditch site then, we
    // assume that it will not be in the ditch site at the end. We assume rational
    // human agents that run *away* from a landing multirotor.
    targets_out_of_view_[n-1] = msg;
  }

  targets_[n-1] = msg;
}

// ----------------------------------------------------------------------------

void MCTrial::step()
{

}

// ----------------------------------------------------------------------------

TrialResult MCTrial::get_results()
{
  TrialResult result;

  // make sure that we don't have any nullptrs
  if (!msg_ds_ || !msg_pose_ || !msg_state_) return result;

  result.complete = true;

  if (landed())
  {
    // std::cout << "Landed!" << std::endl;
    // std::cout << *msg_pose_ << std::endl;

    if (failure())
    {
      std::cout << "Failure!" << std::endl;
      // failure = true;

      result.N_fail = 1;
    }
  }

  return result;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

bool MCTrial::landed()
{
  return (safe2ditch_engaged_ && msg_state_->guided && msg_pose_->pose.position.z <= 5.50);
}

// ----------------------------------------------------------------------------

bool MCTrial::failure()
{

  // ditch site position (in meters)
  auto p_ds = calculate_lla_diff(current_ds_.position.latitude, current_ds_.position.longitude);

  for (int i=0; i<targets_.size(); i++)
  {
    auto target = targets_[i];

    // make sure we don't have a nullptr
    if (!target) continue;

    auto p_ti = std::make_pair(target->pose.pose.position.x, target->pose.pose.position.y);

    // was the final position of the target inside the selected ditch site?
    if (norm(p_ds - p_ti) < current_ds_.radius)
    {
      // double-check: was the target in the ditch site right before
      // the camera FOV was smaller than the ditch site? If not, the
      // chance of noticing the target greatly diminishes, so don't
      // count this as a failure
      auto target_last = targets_out_of_view_[i];
      auto p_ti_last = std::make_pair(target_last->pose.pose.position.x, target_last->pose.pose.position.y);
      if (norm(p_ds - p_ti_last) < current_ds_.radius)
      {
        std::cout << "Target " << i+1 << " " << p_ti << " is inside ditch site " << current_ds_.name << "!" << std::endl;
        return true;
      }

      return false;
    }
    
  }


  return false;
}
 
// ----------------------------------------------------------------------------

std::pair<double,double> MCTrial::calculate_lla_diff(double lat, double lon)
{

  // make sure we don't have a nullptr
  if (!msg_home_) return std::make_pair(0,0);

  //
  // Calculate home UTM
  //

  geographic_msgs::GeoPoint geo_home;
  geo_home.latitude = msg_home_->geo.latitude;
  geo_home.longitude = msg_home_->geo.longitude;
  geo_home.altitude = 0;

  geodesy::UTMPoint utm_home(geo_home);

  //
  // Calculate POI UTM
  //

  geographic_msgs::GeoPoint geo_pt;
  geo_pt.latitude = lat;
  geo_pt.longitude = lon;
  geo_pt.altitude = 0;

  geodesy::UTMPoint utm_pt(geo_pt);

  //
  // Calculate difference in local coords -- meters (WARNING: See assumptions)
  //

  double x = utm_pt.easting - utm_home.easting;
  double y = utm_pt.northing - utm_home.northing;

  return std::make_pair(x,y);
}

}