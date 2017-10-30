#include "drcsim_vehicle_controller.h"

namespace controller
{

Controller::Controller() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{
  // Get Parameters from Server
  nh_private_.param<double>("frequency", freq_, 0.1);
  nh_private_.param<double>("orientation", orient_, 0);
  nh_private_.param<double>("center_x", center_x_, 0);
  nh_private_.param<double>("center_y", center_y_, 0);
  nh_private_.param<double>("size_x", size_x_, 10);
  nh_private_.param<double>("size_y", size_y_, 10);

  // Setup publishers and subscribers
  gas_pedal_sub_  = nh_.subscribe("gas_pedal/state" , 1, &Controller::gasPedalCallback , this);
  hand_brake_sub_ = nh_.subscribe("hand_brake/state", 1, &Controller::handBrakeCallback, this);
  hand_wheel_sub_ = nh_.subscribe("hand_wheel/state", 1, &Controller::handWheelCallback, this);
  odometry_sub_   = nh_.subscribe("odometry", 1, &Controller::odometryCallback, this);

  gas_pedal_pub_  = nh_.advertise<std_msgs::Float64>("gas_pedal/cmd" , 1);
  hand_brake_pub_ = nh_.advertise<std_msgs::Float64>("hand_brake/cmd", 1);
  hand_wheel_pub_ = nh_.advertise<std_msgs::Float64>("hand_wheel/cmd", 1);
  path_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization/path", 1);

  // set variables
  max_speed_ = 4.5;
  max_steer_ = 0.6458;
  omega_ = 2*M_PI*freq_;

  // gas pedal controller parameters
  pid_x_.t_prev = 0;
  pid_x_.integrator = 0;
  pid_x_.err_prev = 0;
  pid_x_.kp = 1.0;
  pid_x_.kd = 0.5;
  pid_x_.ki = 0.1;
  pid_x_.sat = 1.0;

  // steering wheel controller parameters
  pid_psi_.t_prev = 0;
  pid_psi_.integrator = 0;
  pid_psi_.err_prev = 0;
  pid_psi_.kp = 1.0;
  pid_psi_.kd = 0.5;
  pid_psi_.ki = 0.1;
  pid_psi_.sat = max_steer_;

  return;
}


void Controller::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
  // convert quaternion orientation to euler angles
  tf::Quaternion tf_quat;
  double phi, theta, psi;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, tf_quat);
  tf::Matrix3x3(tf_quat).getRPY(phi, theta, psi);

  // get states of the atv
  double pn   = msg->pose.pose.position.x;
  double pw   = msg->pose.pose.position.y;
  double dpn  = msg->twist.twist.linear.x;
  double dpw  = msg->twist.twist.linear.y;
  double dpsi = msg->twist.twist.angular.z;

  // get atv velocities in body frame
  double u = dpn *  cos(psi) + dpw * sin(psi);
  double v = dpn * -sin(psi) + dpw * cos(psi);

  // put values into class variables
  pn_  = pn;
  pw_  = pw;
  u_   = u;
  v_   = v;
  psi_ = psi;
  r_   = dpsi;

  // generate desired path and its derivative (figure 8 for now)
  double t = msg->header.stamp.toSec();
  double pn_r = size_x_ * cos(omega_*t + M_PI/2);
  double pw_r = size_y_ * sin(2*omega_*t);
  double dpn_r = size_x_ * -sin(omega_*t + M_PI/2) * omega_;
  double dpw_r = size_y_ * cos(2*omega_*t) * 2 * omega_;

  // rotate the path
  double pn_r_rot = pn_r *  cos(orient_) + pw_r * sin(orient_);
  double pw_r_rot = pn_r * -sin(orient_) + pw_r * cos(orient_);
  double dpn_r_rot = dpn_r *  cos(orient_) + dpw_r * sin(orient_);
  double dpw_r_rot = dpn_r * -sin(orient_) + dpw_r * cos(orient_);

  // save path to class variables and add offset
  pn_r_ = pn_r_rot + center_x_;
  pw_r_ = pw_r_rot + center_y_;
  dpn_r_ = dpn_r_rot;
  dpw_r_ = dpw_r_rot;

  // publish geolocated objects coordinates to rviz
  visualization_msgs::Marker path_point;
  path_point.type = visualization_msgs::Marker::POINTS;
  path_point.header.stamp = msg->header.stamp;
  path_point.header.frame_id = "world";
  path_point.scale.x = 1.0;
  path_point.scale.y = 1.0;
  path_point.color.r = 0.0f;
  path_point.color.g = 1.0f;
  path_point.color.b = 1.0f;
  path_point.color.a = 1.0;

  geometry_msgs::Point p;
  p.x = pn_r_;
  p.y = pw_r_;
  p.z = 0;
  path_point.points.push_back(p);

  path_pub_.publish(path_point);
}


// this is just to make sure the handbrake stays released
void Controller::handBrakeCallback(const std_msgs::Float64& msg)
{
  // initialize local2 variables
  std_msgs::Float64 outgoing_command;
  double desired_state = 0.0;

  // get current gas pedal state
  double hand_brake_state = msg.data;

  // publish if current state is not at desired value
  if(abs(hand_brake_state-desired_state) > 0.01) {
    outgoing_command.data = desired_state;
    hand_brake_pub_.publish(outgoing_command);
  }

  return;
}


double Controller::computePID(double x, double dx, double xd, double kp, double kd, double ki, double sat, double integrator, double err_prev, double t_prev)
{
  // get time step
  double t_now = ros::Time::now().toSec();
  double dt = t_now - t_prev;
  t_prev = t_now;

  // compute proportional, derivative, and integral parts
  double error = xd - x;
  double proportional = kp * error;
  double differentiator = -kd * dx;
  integrator += ki * 0.5 * (error + err_prev) * dt;

  double u_unsat = proportional + differentiator + integrator;

  // saturate the input
  double u;
  if (u_unsat > sat)
    u = sat;
  else if (u_unsat < -sat)
    u = -sat;
  else
    u = proportional + differentiator + integrator;

  // integrator anti-windup (subtract off saturation)
  if (ki > 0)
    integrator += dt / ki * (u - u_unsat);

  // save error for next integration
  err_prev = error;

  return u;
}


void Controller::gasPedalCallback(const std_msgs::Float64& msg)
{
  // compute the forward perpendicular distance to desired position
  double pn_rel = pn_r_ - pn_; // vehicle frame
  double pw_rel = pw_r_ - pw_;
  double px_rel = pn_rel *  cos(psi_) + pw_rel * sin(psi_); // body frame
  double py_rel = pn_rel * -sin(psi_) + pw_rel * cos(psi_);

  double u_r = dpn_r_ *  cos(psi_) + dpw_r_ * sin(psi_);
  double u_rel = u_r - u_;

  // compute PID command for gas pedal
  double gas = computePID(0, u_, px_rel, pid_x_.kp, pid_x_.kd, pid_x_.ki, pid_x_.sat, pid_x_.integrator, pid_x_.err_prev, pid_x_.t_prev);

  // don't let it stop
  if (px_rel < -0.1)
    gas = 0.1;

  // pack and send the message
  std_msgs::Float64 outgoing_command;
  outgoing_command.data = gas;
  gas_pedal_pub_.publish(outgoing_command);

  return;
}


void Controller::handWheelCallback(const std_msgs::Float64& msg)
{
  // compute the forward perpendicular distance to desired position
  double pn_rel = pn_r_ - pn_; // vehicle frame
  double pw_rel = pw_r_ - pw_;
  double px_rel = pn_rel *  cos(psi_) + pw_rel * sin(psi_); // body frame
  double py_rel = pn_rel * -sin(psi_) + pw_rel * cos(psi_);

  // compute the desired heading
  double psi_rel = atan2(py_rel, px_rel);
  double psi_d = psi_ + psi_rel;

  // compute PID command for gas pedal
  double steer = computePID(psi_, r_, psi_d, pid_psi_.kp, pid_psi_.kd, pid_psi_.ki, pid_psi_.sat, pid_psi_.integrator, pid_psi_.err_prev, pid_psi_.t_prev);

  std_msgs::Float64 outgoing_command;
  outgoing_command.data = steer;
  hand_wheel_pub_.publish(outgoing_command);

  return;
}


} // namespace controller




int main(int argc, char** argv)
{
  ros::init(argc, argv, "drcsim_vehicle_controller");
  controller::Controller Thing;
  ros::spin();

  return 0;
}
