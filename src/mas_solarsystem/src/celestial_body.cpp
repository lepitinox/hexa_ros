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


CelestialBody::CelestialBody() : Node("celestial_body_node",
            rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)
                .automatically_declare_parameters_from_overrides(true))
{
    

    auto update_time = std::chrono::milliseconds(100);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>( "/marker", 10 );
    auto timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&CelestialBody::update, this));
    //auto timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { update(); });

    
    
}
double CelestialBody::calculate_omega(double G, double M, double r) {
    double omega = std::sqrt(G * M / std::pow(r, 3));
    return omega;
}

void CelestialBody::update()
{   auto name = this->get_parameter("name").as_string();
    auto mass = this->get_parameter("Masse").as_double();
    auto orbit_radius = this->get_parameter("Orbite").as_double();
    auto id = this->get_parameter("id").as_int();

    if (name == "Soleil"){
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = name;
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        tf_broadcaster_->sendTransform(t);
    
        // Update the marker.
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = name;
        marker.ns = "";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 1;
        marker.pose.position.y = 1;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 10;
        marker.scale.z = 10;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker_pub_->publish(marker);
//    marker_.frame_locked = true;
    }else{

    auto g_constant_ = 6.67430e-11;
    double angular_velocity_ = calculate_omega(g_constant_, mass, orbit_radius);

    double angle = angular_velocity_ * this->get_clock()->now().seconds();

    double x = orbit_radius * cos(angle);
    double y = orbit_radius * sin(angle);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "Soleil";
    t.child_frame_id = name;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;
    t.transform.rotation.w = 1.0;

    // Broadcast the transform.
    tf_broadcaster_->sendTransform(t);
    
    // Update the marker.
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->get_clock()->now();
    marker.header.frame_id = name;
    marker.ns = "";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 10;
    marker.scale.y = 10;
    marker.scale.z = 10;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_pub_->publish(marker);
//    marker_.frame_locked = true;
    }

    
}