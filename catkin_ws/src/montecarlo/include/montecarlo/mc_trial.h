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

    void step();

    TrialResult get_results();
    
  private:
    int Nt_;      ///< Current number of targets used in this trial
    int m_;       ///< iteration number of this trial

    bool safe2ditch_engaged_ = false;     ///< Has Safe2Ditch been engaged yet?

    nasa_s2d::DitchSite current_ds_;      ///< Currently selected Ditch Site


    geometry_msgs::PoseStamped::ConstPtr msg_pose_;       ///< pose of multirotor
    mavros_msgs::State::ConstPtr msg_state_;              ///< status and mode information of APM
    nasa_s2d::DitchSiteList::ConstPtr msg_ds_;            ///< ditch site list from Safe2Ditch DSS
    mavros_msgs::HomePosition::ConstPtr msg_home_;        ///< position where the multirotor was armed

    std::vector<nav_msgs::Odometry::ConstPtr> targets_;               ///< vector of latest msg for the ith target
    std::vector<nav_msgs::Odometry::ConstPtr> targets_out_of_view_;   ///< vector of latest msg for the ith target before ds out of view

    bool landed();
    bool failure();

    std::pair<double,double> calculate_lla_diff(double lat, double lon);
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