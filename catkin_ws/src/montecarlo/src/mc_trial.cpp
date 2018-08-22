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

  // create a transform for the geolocator
  tf::Transform T;
  tf::poseMsgToTF(msg->pose, T);

  // grab the geolocated corners of the camera field of view
  geolocate_frustum(frustum_corners_, T);

  // update the frustum polygon
  std::vector<Polygon::Point> vertices;
  vertices.push_back( {frustum_corners_(0,0),frustum_corners_(0,1)} );
  vertices.push_back( {frustum_corners_(1,0),frustum_corners_(1,1)} );
  vertices.push_back( {frustum_corners_(2,0),frustum_corners_(2,1)} );
  vertices.push_back( {frustum_corners_(3,0),frustum_corners_(3,1)} );
  frustum_.update(vertices);

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
      // Did Safe2Ditch choose an alternate ditch site?
      if (safe2ditch_engaged_ && current_ds_.name != ds.name)
      {
        // take a snapshot (copy) of the current pose of targets
        targets_reroute_.push_back(targets_);

        // take a snapshot (copy) of the current estimate (tracks) of targets
        tracks3d_reroute_.push_back(msg_tracks3d_);

        // take a snapshot (copy) of the multirotors current pose
        msg_pose_reroute_.push_back(msg_pose_);

        // increase the number of reroutes
        reroutes_++;
      }

      // If the previously selected ditch site is empty then we are now engaged
      if (!safe2ditch_engaged_ && current_ds_.name.empty())
      {
        safe2ditch_engaged_ = true;

        // capture height at Safe2Ditch engagement
        msg_pose_engage_ = msg_pose_;
      }

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
  if (msg_pose_ != nullptr && safe2ditch_engaged_ && msg_pose_->pose.position.z >= OUTSIDE_FOV_ALT)
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

void MCTrial::recv_msg_tracks3d(const visual_mtt::Tracks::ConstPtr& msg)
{
  msg_tracks3d_ = msg;
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

  //
  // Metric: N_fail
  //

  // if the multirotor landed, did it 'fail', i.e., land on someone?
  if (landed())
  {
    if (failure())
    {
      result.N_fail = 1;
    }
  }

  //
  // Metric: time to action
  //

  // How long did it take from when a target was in the FOV until it was 
  // detected, tracked, and then made actionable?
  if (rerouted())
  {

  }

  //
  // Metric: visual false positives causing reroute
  //

  // Did any false positives from the visual system cause a reroute?
  if (rerouted())
  {
    // what is the number of tracks unassociated with true targets that
    // where inside the selected ditch site at the time of reroute?
    result.N_false = number_reroute_false_positives();
  }

  //
  // Metric: altitude at Safe2Ditch engage
  //

  if (safe2ditch_engaged_)
  {
    result.h_engage = msg_pose_engage_->pose.position.z;
  }

  //
  // Metric: altitude at Safe2Ditch reroute
  //

  if (rerouted())
  {
    // Get the multirotor pose from the first reroute.
    // We currently assume that there will only be one reroute
    auto msg_pose = msg_pose_reroute_.front();

    if (msg_pose)
    {
      result.h_reroute = msg_pose->pose.position.z;
    }
  }

  return result;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

bool MCTrial::landed()
{
  return (safe2ditch_engaged_ && msg_state_->guided && msg_pose_->pose.position.z <= LAND_ALT);
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
      if (norm(p_ds - p_ti_last) < current_ds_.radius) return true;
    }
  }


  return false;
}
 
// ----------------------------------------------------------------------------

bool MCTrial::rerouted()
{
  return reroutes_ > 0;
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
  // Calculate difference in local coords -- meters
  //

  // WARNING: This is assuming that we both points are in the same UTM zone,
  // but if they are not, UTM is not continuous! Should use haversine instead
  // see: https://answers.ros.org/question/50763/need-help-converting-lat-long-coordinates-into-meters/?answer=243041#post-id-243041
  double x = utm_pt.easting - utm_home.easting;
  double y = utm_pt.northing - utm_home.northing;

  return std::make_pair(x,y);
}

// ----------------------------------------------------------------------------

int MCTrial::number_reroute_false_positives()
{

  // ditch site position (in meters)
  auto p_ds = calculate_lla_diff(current_ds_.position.latitude, current_ds_.position.longitude);

  // Get the set of target pose snapshots from the first reroute.
  // We currently assume that there will only be one reroute
  auto targets = targets_reroute_.front();
  auto msg_tracks = tracks3d_reroute_.front();

  // std::cout << "True position of Nt targets at reroute:" << std::endl;
  // for (int i=0; i<targets.size(); i++)
  // {
  //   // make sure we don't have a nullptr
  //   if (!targets[i]) continue;

  //   // position of the ith target
  //   auto p_ti = std::make_pair(targets[i]->pose.pose.position.x, targets[i]->pose.pose.position.y);

  //   std::cout << "\t" << p_ti << std::endl;
  // }

  // std::cout << "    ****" << " Target position estimates" << std::endl;
  // for (auto&& track : msg_tracks->tracks)
  // {
  //   auto phat_tj = std::make_pair(track.position.x, track.position.y);

  //   std::cout << "\t" << phat_tj << std::endl;
  // }

  // std::cout << "    ****" << " Nearest Neighbors" << std::endl;
  // auto associations = nearest_neighbors(msg_tracks, targets);
  // for (int i=0; i<associations.size(); i++)
  // {
  //   std::cout << "Target i=" << i << ": " << associations[i] << std::endl;
  // }
  
  int N_unassoc = 0;
  auto associations = nearest_neighbors(msg_tracks, targets, N_unassoc);

  return N_unassoc;
}

// ----------------------------------------------------------------------------

std::vector<int> MCTrial::nearest_neighbors(const visual_mtt::Tracks::ConstPtr& tracks_msg,
    const std::vector<nav_msgs::Odometry::ConstPtr>& targets, int& N_unassoc)
{

  std::vector<visual_mtt::Track> unassociated_tracks = tracks_msg->tracks;

  // the ID at the ith index is associated with the ith target.
  // -1 is used for not currently associated.
  std::vector<int> associations(Nt_, -1);

  // ditch site position (in meters)
  auto p_ds = calculate_lla_diff(current_ds_.position.latitude, current_ds_.position.longitude);


  for (int j=0; j<unassociated_tracks.size(); j++)
  {
    auto track = unassociated_tracks[j];

    // jth position estimate, could be associated with the ith target
    auto phat_tj = std::make_pair(track.position.x, track.position.y);

    // if this track isn't in the ditch site, then it couldn't have triggered a reroute,
    // so don't worry about if it is a track of a true target or not
    if (norm(p_ds - phat_tj) >= current_ds_.radius)
    {
      unassociated_tracks.erase(unassociated_tracks.begin() + j);
      j--;
      continue;
    }



    double smallest_dist = 10.0;
    int smallest_idx = -1;

    for (int i=0; i<targets.size(); i++)
    {
      // make sure we don't have a nullptr
      if (!targets[i]) continue;

      // position of the ith target
      auto p_ti = std::make_pair(targets[i]->pose.pose.position.x, targets[i]->pose.pose.position.y);

      if (norm(p_ti - phat_tj) < smallest_dist)
      {
        smallest_dist = norm(p_ti - phat_tj);
        smallest_idx = i;
      }
    }


    // if we found a smallest distance, associate the target with the track
    if (smallest_idx != -1)
    {
      associations[smallest_idx] = track.id;

      unassociated_tracks.erase(unassociated_tracks.begin() + j);
      j--;
    }
  }

  N_unassoc = unassociated_tracks.size();

  return associations;
}

// ----------------------------------------------------------------------------

void MCTrial::geolocate_frustum(Eigen::MatrixX3d& frustum, const tf::Transform& T)
{
  // Incoming measurements are assumed to be in the normalized image plane

  //
  // cheat and construct the frustum in the nip manually.
  // 

  // a better way would be to just do this in the geolocator node and then
  // record the `/frustum` topic and use that here.

  double width = 800;
  double height = 600;

  // got these values from /camera/camera_info topic
  Eigen::Vector3d f = Eigen::Vector3d(1308.7504841320101,1308.7504841320101,1);
  Eigen::Vector3d c = Eigen::Vector3d(400.5,300.5,0);
  
  // create the normalized image plane coordinates of the four corners as rows
  frustum = Eigen::MatrixX3d::Zero(4,3);
  frustum.row(0) = (Eigen::Vector3d(0,0,1).array() - c.array())/f.array();
  frustum.row(1) = (Eigen::Vector3d(width,0,1).array() - c.array())/f.array();
  frustum.row(2) = (Eigen::Vector3d(width,height,1).array() - c.array())/f.array();
  frustum.row(3) = (Eigen::Vector3d(0,height,1).array() - c.array())/f.array();

  ///////////////////////////////////////////////////////////////////////////

  // how many measurements are there?
  uint32_t N = frustum.rows();

  // UAV Position
  double pn = T.getOrigin().x();
  double pe = T.getOrigin().y();
  double pd = T.getOrigin().z();

  // ------------------------------------------------------------------------
  // Compute equation (13.9) in UAV book      (pts == ell_unit_c)
  // ------------------------------------------------------------------------

  // norm each row of the pts matrix
  Eigen::VectorXd F = frustum.rowwise().norm();

  // Make the Nx1 vector an Nx3 matrix so we can element-wise divide
  Eigen::MatrixX3d Fdiv = F.replicate(1, 3);

  // divide to normalize and create unit vectors in the camera frame
  Eigen::MatrixXd pts = (frustum.array() / Fdiv.array());

  // ========================================================================


  // Create the rotation from camera frame to vehicle frame
  Eigen::Affine3d e;
  tf::transformTFToEigen(T, e);
  Eigen::Matrix3d R_c_to_v = e.rotation();

  // Rotate camera frame unit vectors (ell_unit_c) into vehicle frame
  // (see the numerator of RHS of (13.18) in UAV book)
  Eigen::Matrix3Xd ell_unit_v = R_c_to_v * pts.transpose();

  // ------------------------------------------------------------------------
  // Compute equation (13.17) in UAV book     (target's range estimate)
  // ------------------------------------------------------------------------

  // cosine of the angle between the the ki axis and ell_unit_v
  // (the unit vector that points at each target)
  //
  // Instead of creating a ki_unit << (0 0 1) and doing a dot product,
  // just grab the last row of ell_unit_v, since that's what
  // <ki_unit, ell_unit_v> would have done anyways.
  Eigen::VectorXd cos_psi = ell_unit_v.row(2);

  // The UAV's height above ground (well, actually the gimbal/camera)
  float h = -pd;

  // 1 x N vector
  Eigen::VectorXd L = h / cos_psi.array();

  // ========================================================================


  // ------------------------------------------------------------------------
  // Compute equation (13.18) in UAV book   (target's pos in intertial frame)
  // ------------------------------------------------------------------------

  // Construct a vector for the UAV's inertial position
  Eigen::Vector3d tmp; tmp << pn, pe, pd;
  Eigen::Matrix3Xd P_mav_i = tmp.replicate(1, N);

  // Replicate L vector to a matrix so dimensions are happy
  Eigen::Matrix3Xd Lmat = L.transpose().replicate(3, 1);

  // Based on current attitude, find offset from the body to vehicle frame.
  // This is because the camera (on the gimbal) sees targets, not the UAV.
  // Mat offset = rot_v2b(phi,theta,psi).t() * cv::repeat(d_b2g(), 1, N);
  Eigen::Matrix3Xd offset = Eigen::Matrix3Xd::Zero(3, N);

  // Based on the line-of-sight vector, find the inertial object points
  Eigen::Matrix3Xd P_obj_i = P_mav_i + offset + (Lmat.array() * ell_unit_v.array()).matrix();

  // ========================================================================

  // Copy object points back to measurements for the caller
  frustum = P_obj_i.transpose();
}

}