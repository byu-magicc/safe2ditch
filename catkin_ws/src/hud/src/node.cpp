#include <ros/ros.h>
#include "hud/hud.h"

int main(int argc, char** argv) {
  // start node
  ros::init(argc, argv, "hud");

  // instantiate an object
  hud::HUD hud;

  ros::spin();
  return 0;
}