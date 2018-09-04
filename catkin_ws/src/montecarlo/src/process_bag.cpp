/**
 * @file process_bag.h
 * @author Parker Lusk <parkerclusk@gmail.com>
 */

// #include <ros/ros.h>

// #include <opencv2/opencv.hpp>

// #include <ros/ros.h>
// #include <rosbag/bag.h>
// #include <rosbag/view.h>

// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>

// #include <std_msgs/Header.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/CompressedImage.h>
// #include <sensor_msgs/image_encodings.h>

// #include <boost/format.hpp>
// #include <boost/foreach.hpp>

#include "argparse/argparse.hpp"

#include "montecarlo/mc_processor.h"


int main(int argc, const char **argv)
{

  //
  // Argument Parsing
  //

  ArgumentParser parser;

  // add some arguments to search for
  parser.addArgument("-d", "--directory", 1);     // directory where simulation bags are located

  // parse the command-line arguments -- exits if invalid format
  parser.parse(argc, argv);

  // do some extra parsing on the directory and prepend format
  std::string dir = (parser.count("directory")>0) ? parser.retrieve<std::string>("directory") : ".";

  //
  // Kick off the Monte Carlo processor
  //

  montecarlo::MCProcessor processor(dir);

  processor.start();

  return 0;



  std::string bag_path = "/home/plusk01/dev/safe2ditch/catkin_ws/src/montecarlo/scripts/mcsim_18Aug2018_t7_m1.bag";

  // display what we've parsed
  std::cout << std::endl;
  std::cout << "Opening bag:\t\t" << bag_path << std::endl;
  // std::cout << "Listening to topic:\t" << parser.retrieve<std::string>("topic") << std::endl;
  // std::cout << "Saving to:\t\t" << dir + "/" + prepend + "*" << std::endl;

  std::cout << std::endl << std::endl << "************************************" << std::endl;
  std::cout << "Current Header:" << std::endl << std::endl;

  //
  // timestamp.xml setup
  //

  // std::ofstream xmlFile;
  // xmlFile.open(dir + "/" + "timestamp.xml");

  // Writer xml(xmlFile);

  // xml.openElt("extractor");

  // xml.openElt("info");
  // xml.openElt("bag").content(parser.retrieve<std::string>("bag").c_str()).closeElt();
  // xml.openElt("topic").content(parser.retrieve<std::string>("topic").c_str()).closeElt();
  // if (duration > 0)
  //   xml.openElt("duration").content(parser.retrieve<std::string>("secs").c_str()).closeElt();
  // xml.closeElt();

  // xml.openElt("timestamps");

  //
  // rosbag setup
  //

  // rosbag::Bag bag;
  // bag.open(bag_path, rosbag::bagmode::Read);

  // std::vector<std::string> topics;
  // topics.push_back("/hud/image_raw/compressed");

  // // Path and filename setup
  // boost::format filename_format;
  // filename_format.parse(dir + "/" + prepend + "%i.jpg");

  // // Create a view into the rosbag using the topics
  // rosbag::View view(bag, rosbag::TopicQuery(topics));

  // // The first header.seq may not be 0. In that case, shift everything so that the
  // // first frame is written as frame 0 even though it may not be
  // int first_frame = -1;

  // // Loop through each message in the view, deciding if it is
  // // a compressed or raw image and handling appropriately.
  // BOOST_FOREACH(rosbag::MessageInstance const m, view)
  // {

  //   // A place to store the image from the message
  //   cv::Mat image;
  //   int frame_number = 0;
  //   std_msgs::Header header;

  //   //
  //   // Image messages
  //   //

  //   sensor_msgs::Image::ConstPtr msg_image = m.instantiate<sensor_msgs::Image>();
  //   if (msg_image != nullptr)
  //   {
  //     header = msg_image->header;
      
  //     image = cv_bridge::toCvShare(msg_image, "bgr8")->image;
  //   }

  //   //
  //   // Compressed Image messages
  //   //

  //   sensor_msgs::CompressedImage::ConstPtr msg_compressed = m.instantiate<sensor_msgs::CompressedImage>();
  //   if (msg_compressed != nullptr)
  //   {
  //     header = msg_compressed->header;

  //     image = cv::imdecode(cv::Mat(msg_compressed->data), cv::IMREAD_UNCHANGED);
  //   }

  //   // Capture the first frame number in case we need to offset everything
  //   if (first_frame == -1) first_frame = header.seq;

  //   // Build the filename based on what frame this is
  //   frame_number = header.seq - first_frame;
  //   std::string filename = (filename_format % frame_number).str();

  //   // Write the image to file
  //   cv::imwrite(filename, image);

  //   // Display the current message header
  //   std::cout << header;

  //   // Keep displaying in the same place. The tabs are to clear
  //   // anything left over on the first line to keep it clean.
  //   std::cout << "\e[A\e[A\e[A\t\t\t\t\t\r";

  //   // // Write the frame and timestamp to the XML file
  //   // xml.openElt("frame");
  //   // xml.openElt("num").content(std::to_string(frame_number).c_str()).closeElt();
  //   // xml.openElt("t").content(std::to_string(header.stamp.toSec()).c_str()).closeElt();
  //   // xml.closeElt();

  //   // If there was a duration specified, should I bail?
  //   if (duration > 0 && (header.stamp - view.getBeginTime()).toSec() >= duration)
  //     break;
  // }

  // // Move past the header text
  // std::cout << "\n\n\n\n";
  // // xml.closeAll();
  // // xmlFile.close();
  // bag.close();
  // return 0;
}