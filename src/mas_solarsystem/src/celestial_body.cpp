#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "celestial_body.h"
#include <cmath>
#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"


CelestialBody::CelestialBody(
  const std::string &name,
  double mass,
  std::vector<double> position,
  std::vector<double> velocity,
  double g_constant,
  double orbit_radius) : Node(name),
    name_(name),
    mass_(mass),
    position_(position),
    velocity_(velocity),
    g_constant_(g_constant),
    orbit_radius_(orbit_radius){

    auto update_time = std::chrono::milliseconds(10);
    auto timer_ = create_wall_timer(update_time, std::bind(&CelestialBody::update, this));
    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::msg::Marker>( "visualization_marker", 0 );

}
double CelestialBody::calculate_omega(double G, double M, double r) {
    double omega = std::sqrt(G * M / std::pow(r, 3));
    return omega;
}

void CelestialBody::update()
{
    auto g_constant_ = 6.67430e-11;
    double angular_velocity_ = calculate_omega(g_constant_, mass_, orbit_radius_);
    double angle = timer_ * angular_velocity_;

    double x = orbit_radius_ * cos(angle);
    double y = orbit_radius_ * sin(angle);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "sun";
    t.child_frame_id = get_name();
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;
    t.transform.rotation.w = 1.0;

    // Broadcast the transform.
    tf_broadcaster_->sendTransform(t);

    // Update the marker.
    marker_.header.stamp = this->get_clock()->now();
    marker_.pose.position.x = x;
    marker_.pose.position.y = y;
    marker_.pose.position.z = 0.0;
    marker_.pose.orientation.w = 1.0;
    marker_.frame_locked = true;

    vis_pub.publish(marker_);
    
}