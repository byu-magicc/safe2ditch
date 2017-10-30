#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

#define _USE_MATH_DEFINES

using namespace std;

namespace controller
{



typedef struct
{
  double integrator;
  double t_prev;
  double err_prev;
  double kp;
  double kd;
  double ki;
  double sat;
}pid_t;



class Controller
{

public:

  Controller();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Subscribers
  ros::Subscriber gas_pedal_sub_;
  ros::Subscriber hand_brake_sub_;
  ros::Subscriber hand_wheel_sub_;
  ros::Subscriber odometry_sub_;

  ros::Publisher gas_pedal_pub_;
  ros::Publisher hand_brake_pub_;
  ros::Publisher hand_wheel_pub_;
  ros::Publisher path_pub_;

  // Variables
  double max_speed_, max_steer_; // vehicle parameters
  double freq_, orient_, omega_, center_x_, center_y_, size_x_, size_y_; // path parameters
  double pn_, pw_, u_, v_, psi_, r_; // vehicle states
  double pn_r_, pw_r_, dpn_r_, dpw_r_; // trajectory reference positions
  pid_t pid_x_, pid_psi_; // for pid controllers

  // Functions
  void gasPedalCallback(const std_msgs::Float64 &msg);
  void handBrakeCallback(const std_msgs::Float64 &msg);
  void handWheelCallback(const std_msgs::Float64 &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  double computePID(double x, double dx, double xd, double kp, double kd, double ki, double sat, double integrator, double err_prev, double t_prev);
};

} // namespace controller

#endif // VEHICLE_CONTROLLER_H
