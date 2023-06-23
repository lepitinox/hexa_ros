#pragma once

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"

class CelestialBody: public rclcpp::Node(name),
    name_(name),
    mass_(mass),
    position_(position),
    velocity_(velocity),
    g_constant_(g_constant),
    orbit_radius_(orbit_radius)
{
public:
// constructor
  CelestialBody(
    const std::string &name,
    double mass,
    std::vector<double> position,
    std::vector<double> velocity,
    double g_constant = 6.67430e-11,
    double orbit_radius = 0.0);

  void update();

private:

  std::string name_;
  double mass_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  double g_constant_;
  int marker_id_;
  double orbit_radius_;   

  rclcpp::Clock::SharedPtr clock_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  visualization_msgs::msg::Marker marker_;};