/**
 * @file mc_trial.h
 * @author Parker Lusk <parkerclusk@gmail.com>
 */

#pragma once

#include <string>
#include <vector>
#include <utility>

#include <geodesy/utm.h>

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>

#include <nasa_s2d/DitchSiteList.h>
#include <nasa_s2d/DitchSite.h>

#include <visual_mtt/Tracks.h>
#include <visual_mtt/Track.h>

namespace montecarlo {

  struct TrialResult
  {
    double t_action = 0;
    double N_fail = 0;
    double N_false = 0;

    bool complete = false;

    TrialResult average(int total) {
      return (*this)/total;
    }

    template <typename T>
    friend TrialResult operator/(TrialResult lhs, const T& rhs) {
      lhs.t_action /= rhs;
      lhs.N_fail /= rhs;
      lhs.N_false /= rhs;
      return lhs;
    }

    friend TrialResult operator+(TrialResult lhs, const TrialResult& rhs) {
      lhs += rhs;
      return lhs;
    }

    TrialResult& operator+=(const TrialResult& other) {
      t_action += other.t_action;
      N_fail += other.N_fail;
      N_false += other.N_false;
      complete = complete || other.complete;
      return *this;
    }

    friend std::ostream& operator<<(std::ostream& out, const TrialResult& result) {
      out << "Result: <" << result.t_action << ", " << result.N_fail << ", " << result.N_false << ">";
      return out;
    }
  };

  // --------------------------------------------------------------------------

  class MCTrial
  {
  public:
    MCTrial(int Nt, int m);
    ~MCTrial() = default;

    void recv_msg_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void recv_msg_state(const mavros_msgs::State::ConstPtr& msg);
    void recv_msg_ditchsites(const nasa_s2d::DitchSiteList::ConstPtr& msg);
    void recv_msg_home(const mavros_msgs::HomePosition::ConstPtr& msg);
    void recv_msg_target(int n, const nav_msgs::Odometry::ConstPtr& msg);
    void recv_msg_tracks3d(const visual_mtt::Tracks::ConstPtr& msg);

    void step();

    TrialResult get_results();
    
  private:
    int Nt_;      ///< Current number of targets used in this trial
    int m_;       ///< iteration number of this trial

    bool safe2ditch_engaged_ = false;     ///< Has Safe2Ditch been engaged yet?
    int reroutes_ = 0;                    ///< How many times did Safe2Ditch choose alternate ditch sites?

    nasa_s2d::DitchSite current_ds_;      ///< Currently selected Ditch Site


    static constexpr double LAND_ALT = 5.50;          ///< consider multirotor to have landed below this altitude
    static constexpr double OUTSIDE_FOV_ALT = 15.0;   ///< ditch site roughly fills up entire camera FOV at this altitude


    geometry_msgs::PoseStamped::ConstPtr msg_pose_;       ///< pose of multirotor
    mavros_msgs::State::ConstPtr msg_state_;              ///< status and mode information of APM
    nasa_s2d::DitchSiteList::ConstPtr msg_ds_;            ///< ditch site list from Safe2Ditch DSS
    mavros_msgs::HomePosition::ConstPtr msg_home_;        ///< position where the multirotor was armed
    visual_mtt::Tracks::ConstPtr msg_tracks3d_;           ///< estimate of geolocated target positions from Visual MTT
    std::vector<visual_mtt::Tracks::ConstPtr> tracks3d_reroute_;  ///< estimate of geolocated target positions from Visual MTT, at time of reroute

    std::vector<nav_msgs::Odometry::ConstPtr> targets_;               ///< vector of latest msg for the ith target
    std::vector<nav_msgs::Odometry::ConstPtr> targets_out_of_view_;   ///< vector of latest msg for the ith target before ds out of view
    std::vector<std::vector<nav_msgs::Odometry::ConstPtr>> targets_reroute_; ///< pose of jth target when Safe2Ditch rerouted the ith time

    /**
     * @brief      At the end of the trial, did the multirotor land?
     *
     * @return     true if final altitude <= LAND_ALT
     */
    bool landed();

    /**
     * @brief      Did the multirotor landed on an obstacle?
     *
     * @return     bool
     */
    bool failure();

    /**
     * @brief      Did Safe2Ditch reroute to an alternate ditch site?
     *
     * @return     bool
     */
    bool rerouted();

    std::pair<double,double> calculate_lla_diff(double lat, double lon);

    int number_reroute_false_positives();

    std::vector<int> nearest_neighbors(const visual_mtt::Tracks::ConstPtr& tracks_msg, const std::vector<nav_msgs::Odometry::ConstPtr>& targets, int& N_unassoc);
  };

  // --------------------------------------------------------------------------

  // printing a pair
  template <typename T>
  std::ostream& operator<<(std::ostream& out, const std::pair<T,T>& x) {
    out << "(" << x.first << ", " << x.second << ")";
    return out;
  }

  // Support pair addition
  template <typename T>
  std::pair<T,T> operator+(const std::pair<T,T>& lhs, const std::pair<T,T>& rhs) {
      return std::make_pair(lhs.first+rhs.first, lhs.second+rhs.second);
  }

  // Support pair differencing
  template <typename T>
  std::pair<T,T> operator-(const std::pair<T,T>& lhs, const std::pair<T,T>& rhs) {
      return std::make_pair(lhs.first-rhs.first, lhs.second-rhs.second);
  }

  // calculate the magnitude of a waypoint
  template <typename T>
  double norm(std::pair<T,T> x) {
      return sqrt(pow(x.first,2) + pow(x.second,2));
  }

}