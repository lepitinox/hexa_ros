#pragma once

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"

class CelestialBody : public rclcpp::Node

{
public:
// constructor
  CelestialBody();

  void update();
  double calculate_omega(double G, double M, double r);
  rclcpp::TimerBase::SharedPtr timer_;
private:

  std::string name_;
  double mass_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  double g_constant_;
  int marker_id_;
  double orbit_radius_;
  std::string name;
  double mass;
  double orbit_radius;
  int id;
  double Rayon;
  double r_scale;
  double d_scale;

  rclcpp::Clock::SharedPtr clock_;
  geometry_msgs::msg::TransformStamped t;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  


};