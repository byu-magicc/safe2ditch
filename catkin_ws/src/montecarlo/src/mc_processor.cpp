  /**
 * @file mc_processor.cpp
 * @author Parker Lusk <parkerclusk@gmail.com>
 */

#include "montecarlo/mc_processor.h"

#include <iostream>
#include <vector>
#include <algorithm>

#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <std_msgs/Header.h>

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>

#include <nasa_s2d/DitchSiteList.h>
#include <nasa_s2d/DitchSite.h>

#include <visual_mtt/Tracks.h>
#include <visual_mtt/Track.h>

namespace montecarlo {

MCProcessor::MCProcessor(std::string bagdir)
: bagdir_(bagdir)
{
  // find the MC trial bags and get overall Monte Carlo stats based on ros bags
  bags_ = find_bags();
  get_mc_stats(bags_, Nts_, M_);
}

// ----------------------------------------------------------------------------

void MCProcessor::start()
{
  // Print out Monte Carlo stats
  std::cout << std::string(80, '*') << std::endl;
  std::cout << "* List of Nt: [";
  for (int i=0; i<Nts_.size(); i++)
  {
    std::cout << Nts_[i];
    if (i != Nts_.size()-1) std::cout << ", ";
  }
  std::cout << "]" << std::endl;
  std::cout << "* Trials per Nt: " << M_ << std::endl;
  std::cout << std::string(80, '*') << std::endl;

  for_each_Nt();
}

// ----------------------------------------------------------------------------
// Private Methods
// -----------------------------------------------------------------------------

std::vector<Bag> MCProcessor::find_bags()
{
  const boost::regex re("[[:word:]]*t([0-9]+)_m([0-9]+)\\.bag");

  std::vector<Bag> bags;

  std::cout << "Finding bags..." << std::endl;

  // https://stackoverflow.com/a/1259198/2392520
  boost::filesystem::directory_iterator end_itr; // Default ctor yields past-the-end
  for (boost::filesystem::directory_iterator i(bagdir_); i != end_itr; ++i)
  {
    boost::smatch what;

    // Skip if not a file
    if (!boost::filesystem::is_regular_file(i->status())) continue;

    // Skip if not a match
    if (!boost::regex_search(i->path().filename().string(), what, re)) continue;

    // extract the Nt and m
    std::string t(what[1].first, what[1].second);
    std::string m(what[2].first, what[2].second);

    // File matches, store it
    bags.push_back(std::make_tuple(i->path().filename().string(), stoi(t), stoi(m)));
  }

  std::sort(bags.begin(), bags.end(),
    [](const Bag& b1, const Bag& b2) {
      if (std::get<1>(b1) < std::get<1>(b2)) {
        return true;
      } else if (std::get<1>(b1) == std::get<1>(b2)) {
        return (std::get<2>(b1) < std::get<2>(b2));
      } else {
        return false;
      }
    });

  // for (auto&& bag : bags)
  // {
  //   std::cout << "Match: " << std::get<0>(bag) << "\tt" << std::get<1>(bag) << "_m" << std::get<2>(bag) << std::endl;
  // }

  return bags;
}
 
// -----------------------------------------------------------------------------

void MCProcessor::get_mc_stats(const std::vector<Bag>& bags, std::vector<int>& Nts, int& M)
{
  Nts.clear();
  M = 0;

  for (auto&& bag : bags)
  {
    int Nt = std::get<1>(bag);
    int m = std::get<2>(bag);

    // make a list of each 'number of targets', Nt
    if (std::find(Nts.begin(), Nts.end(), Nt) == Nts.end()) Nts.push_back(Nt);

    // find the highest trial iteration number
    if (m > M) M = m;
  }
}

// -----------------------------------------------------------------------------

void MCProcessor::for_each_Nt()
{

  int Nt = 0;
  int m = 0;

  TrialResult total;

  // we assume that bags_ is already sorted
  for (auto&& bag : bags_)
  {
    // Update Nt and m if needed
    if (m != std::get<2>(bag)) m = std::get<2>(bag);
    if (Nt != std::get<1>(bag))
    {
      Nt = std::get<1>(bag);
      std::cout << std::string(35, '-') <<  " Nt = " << Nt << " " << std::string(35, '-') << std::endl;
    }

    std::cout << "Processing t" << Nt << "_m" << m << " (" << std::get<0>(bag) << "):" << std::flush;
    auto result = process_trial(bagdir_ + std::get<0>(bag), Nt, m);
    std::cout << result << std::endl;

    total += result;

    // early termination
    if (Nt == 1 && m == 100) break;
    
    if (m == 100) std::cout << total << std::endl;
  }

  std::cout << total << std::endl;
}

// ----------------------------------------------------------------------------

TrialResult MCProcessor::process_trial(std::string bagpath, int Nt, int m)
{
  //
  // Create a trial object
  //

  MCTrial trial(Nt, m);

  //
  // rosbag setup
  //

  rosbag::Bag bag;
  bag.open(bagpath, rosbag::bagmode::Read);

  // std::vector<std::string> topics;
  // topics.push_back("/hud/image_raw/compressed");
  // topics.push_back("/mavros/local_position/pose");
  // topics.push_back("/mavros/state");
  // topics.push_back("/dss/ditch_sites");

  // Create a view into the rosbag using the topics
  // rosbag::View view(bag, rosbag::TopicQuery(topics));
  rosbag::View view(bag);

  // Loop through each message in the view, deciding if it is
  // a compressed or raw image and handling appropriately.
  BOOST_FOREACH(const rosbag::MessageInstance& mm, view)
  {

    //
    // Pose
    //

    geometry_msgs::PoseStamped::ConstPtr msg_pose = mm.instantiate<geometry_msgs::PoseStamped>();
    if (msg_pose != nullptr && mm.getTopic() == "/mavros/local_position/pose")
    {
      trial.recv_msg_pose(msg_pose);
      trial.step();
    }

    //
    // DitchSiteList
    //
    
    nasa_s2d::DitchSiteList::ConstPtr msg_ds_list = mm.instantiate<nasa_s2d::DitchSiteList>();
    if (msg_ds_list != nullptr && mm.getTopic() == "/dss/ditch_sites")
    {
      trial.recv_msg_ditchsites(msg_ds_list);
    }

    //
    // State
    //

    mavros_msgs::State::ConstPtr msg_state = mm.instantiate<mavros_msgs::State>();
    if (msg_state != nullptr && mm.getTopic() == "/mavros/state")
    {
      trial.recv_msg_state(msg_state);
    }

    //
    // Home
    //

    mavros_msgs::HomePosition::ConstPtr msg_home = mm.instantiate<mavros_msgs::HomePosition>();
    if (msg_home != nullptr && mm.getTopic() == "/mavros/home_position/home")
    {
      trial.recv_msg_home(msg_home);
    }

    //
    // Target Pose (truth)
    //

    nav_msgs::Odometry::ConstPtr msg_target = mm.instantiate<nav_msgs::Odometry>();
    if (msg_target != nullptr && mm.getTopic().find("/targets") != std::string::npos)
    {
      const boost::regex re("/targets/target([[:digit:]]+)/state");

      boost::smatch what;

      // Skip if not a match
      if (boost::regex_search(mm.getTopic(), what, re))
      {
        // extract the target number
        std::string target_num(what[1].first, what[1].second);

        trial.recv_msg_target(stoi(target_num), msg_target);
      }
    }

    //
    // Tracks (estimate of target positions)
    //

    visual_mtt::Tracks::ConstPtr msg_tracks3d = mm.instantiate<visual_mtt::Tracks>();
    if (msg_tracks3d != nullptr && mm.getTopic() == "/tracks3d")
    {
      trial.recv_msg_tracks3d(msg_tracks3d);
    }
    
  }

  return trial.get_results();
}

// ----------------------------------------------------------------------------


}
