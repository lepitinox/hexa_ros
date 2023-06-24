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
    // log in the console that the node is starting
    RCLCPP_INFO(this->get_logger(), "Starting Celestial Body Node");
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>( "/marker", 10 );
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "OKLOL 2");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&CelestialBody::update, this));
    
    //auto timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { update(); });

    this->name = this->get_parameter("name").as_string();
    this->mass = this->get_parameter("Masse").as_double();
    this->orbit_radius = this->get_parameter("Orbite").as_int();
    this->id = this->get_parameter("id").as_int();

}
double CelestialBody::calculate_omega(double G, double M, double r) {
    double omega = std::sqrt(G * M / std::pow(r, 3));
    return omega;
}

void CelestialBody::update()
{   
    RCLCPP_INFO(this->get_logger(), "In update");
    auto name = this->name;
    auto mass = this->mass;
    auto orbit_radius = this->orbit_radius;
    auto id = this->id;


    RCLCPP_INFO(this->get_logger(), "name: %s", name.c_str());

    if (name == "Soleil"){
        RCLCPP_INFO(this->get_logger(), "In Soleil");
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = name;
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 0.0;
        RCLCPP_INFO(this->get_logger(), "Soleil: transfomr up");
                // Update the marker.
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = this->name;
        marker.ns = "";
        marker.id = this->id;
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
        marker.color.a = 1.0; 
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        RCLCPP_INFO(this->get_logger(), "Soleil: marker up");
        marker.frame_locked = true;
        marker_pub_->publish(marker);
    


        RCLCPP_INFO(this->get_logger(), "Soleil: marker_pub_");
        tf_broadcaster_->sendTransform(t);
        RCLCPP_INFO(this->get_logger(), "Soleil: tf_broadcaster_");

    }else{
    RCLCPP_INFO(this->get_logger(), "In planet");
    auto g_constant_ = 6.67430e-11;
    double angular_velocity_ = calculate_omega(g_constant_, mass, orbit_radius);
    RCLCPP_INFO(this->get_logger(), "In planet: angular done");
    double angle = angular_velocity_ * this->get_clock()->now().seconds();
    RCLCPP_INFO(this->get_logger(), "In planet: angle done");
    double x = orbit_radius * cos(angle);
    double y = orbit_radius * sin(angle);
    RCLCPP_INFO(this->get_logger(), "In planet: sin cos done");
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "Soleil";
    t.child_frame_id = name;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 0.0;
    RCLCPP_INFO(this->get_logger(), "planet: transform up");
    // Broadcast the transform.
    
    
    // Update the marker.
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->get_clock()->now();
    marker.header.frame_id = name;
    marker.ns = "";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 10;
    marker.scale.y = 10;
    marker.scale.z = 10;
    marker.color.a = 1.0; 
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    RCLCPP_INFO(this->get_logger(), "planet: marker up");
    marker.frame_locked = true;
    marker_pub_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "planet: marker_pub_");
    tf_broadcaster_->sendTransform(t);
    RCLCPP_INFO(this->get_logger(), "planet:tf_broadcaster_");

    }
 }