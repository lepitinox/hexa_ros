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
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>( "/marker", 1000 );
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "OKLOL 2");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&CelestialBody::update, this));
    
    //auto timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { update(); });

    this->name = this->get_parameter("name").as_string();
    this->mass = this->get_parameter("Masse").as_double();
    this->orbit_radius = this->get_parameter("Orbite").as_int();
    this->id = this->get_parameter("id").as_int();
    this->Rayon = this->get_parameter("Rayon").as_double();
    this->r_scale = this->get_parameter("r_scale").as_double();
    this->d_scale = this->get_parameter("d_scale").as_double();

}
double CelestialBody::calculate_omega(double G, double M, double r) {
    double omega = std::sqrt(G * M / std::pow(r, 3));
    return omega;
}

void CelestialBody::update()
{   
    if (name == "Soleil"){

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = this->name;
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0;
        t.transform.rotation.y = 0;
        t.transform.rotation.z = 0;
        t.transform.rotation.w = 1;
        tf_broadcaster_->sendTransform(t);

        // Update the marker.
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = this->name;
        marker.ns = "";
        marker.id = this->id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = this->Rayon * this->r_scale;
        marker.scale.y = this->Rayon * this->r_scale;
        marker.scale.z = this->Rayon * this->r_scale;
        marker.color.a = 1.0; 
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.frame_locked = true;

        marker_pub_->publish(marker);
        

    }else{

    auto g_constant_ = 6.67430e-11;
    double angular_velocity_ = calculate_omega(g_constant_, this->mass, this->orbit_radius);

    double angle = angular_velocity_ * this->get_clock()->now().seconds();

    double x = orbit_radius * cos(angle);
    double y = orbit_radius * sin(angle);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "Soleil";
    t.child_frame_id = name;
    t.transform.translation.x = x * this->d_scale;
    t.transform.translation.y = y * this->d_scale;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 5);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
    
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->get_clock()->now();
    marker.header.frame_id = this->name;
    marker.ns = "";
    marker.id = this->id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = this->Rayon * this->r_scale;
    marker.scale.y = this->Rayon * this->r_scale;
    marker.scale.z = this->Rayon * this->r_scale;
    marker.color.a = 1.0; 
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.frame_locked = true;
    marker_pub_->publish(marker);
    

    }
 }