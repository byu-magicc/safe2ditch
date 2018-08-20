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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

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
  for (auto&& Nt : Nts_)
  {
    for (int m=1; m<=M_; m++)
    {

      std::cout << "t" << Nt << "_m" << m << std::endl;

      auto it = std::find_if(bags_.begin(), bags_.end(),
        [&Nt, m](const Bag& bag) {
          return (std::get<1>(bag) == Nt) && std::get<2>(bag) == m;
        });

      std::string bagpath = std::get<0>(*it);

      // std::string bagpath = "mcsim_18Aug2018_t1_m1.bag";

      process_trial(bagdir_ + bagpath, Nt, m);

    }
  }
}

// ----------------------------------------------------------------------------

void MCProcessor::process_trial(std::string bagpath, int Nt, int m)
{

  //
  // rosbag setup
  //

  rosbag::Bag bag;
  bag.open(bagpath, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back("/hud/image_raw/compressed");
  topics.push_back("/mavros/local_position/pose");
  topics.push_back("/mavros/state");
  topics.push_back("/clock");

  // Create a view into the rosbag using the topics
  rosbag::View view(bag, rosbag::TopicQuery(topics));


  // Loop through each message in the view, deciding if it is
  // a compressed or raw image and handling appropriately.
  BOOST_FOREACH(const rosbag::MessageInstance& msg, view)
  {

    // // A place to store the image from the message
    // cv::Mat image;
    // int frame_number = 0;
    // std_msgs::Header header;

    // //
    // // Image messages
    // //

    // sensor_msgs::Image::ConstPtr msg_image = m.instantiate<sensor_msgs::Image>();
    // if (msg_image != nullptr)
    // {
    //   header = msg_image->header;
      
    //   image = cv_bridge::toCvShare(msg_image, "bgr8")->image;
    // }

    // //
    // // Compressed Image messages
    // //

    sensor_msgs::CompressedImage::ConstPtr msg_compressed = msg.instantiate<sensor_msgs::CompressedImage>();
    if (msg_compressed != nullptr)
    {
      auto header = msg_compressed->header;
      // std::cout << "\t" << header.seq << std::endl;

    }

    // // Capture the first frame number in case we need to offset everything
    // if (first_frame == -1) first_frame = header.seq;

    // // Build the filename based on what frame this is
    // frame_number = header.seq - first_frame;
    // std::string filename = (filename_format % frame_number).str();

    // // Write the image to file
    // cv::imwrite(filename, image);

    // // Display the current message header
    // std::cout << header;

    // // Keep displaying in the same place. The tabs are to clear
    // // anything left over on the first line to keep it clean.
    // std::cout << "\e[A\e[A\e[A\t\t\t\t\t\r";

    // // // Write the frame and timestamp to the XML file
    // // xml.openElt("frame");
    // // xml.openElt("num").content(std::to_string(frame_number).c_str()).closeElt();
    // // xml.openElt("t").content(std::to_string(header.stamp.toSec()).c_str()).closeElt();
    // // xml.closeElt();

    // // If there was a duration specified, should I bail?
    // if (duration > 0 && (header.stamp - view.getBeginTime()).toSec() >= duration)
    //   break;
  }

}

}
