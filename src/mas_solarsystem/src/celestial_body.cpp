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

class CelestialBody : public rclcpp::Node
{
public:
    CelestialBody(
        const std::string &name,
        double mass,
        std::vector<double> position,
        std::vector<double> velocity,
        double g_constant,
        double orbit_radius)
        : rclcpp::Node(name),
          name_(name),
          mass_(mass),
          position_(position),
          velocity_(velocity),
          g_constant_(g_constant),
          orbit_radius_(orbit_radius)
    {
        auto update_time = std::chrono::milliseconds(10);
        timer_ = create_wall_timer(update_time, std::bind(&CelestialBody::update, this));
        vis_pub = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    }

private:
    double calculate_omega(double G, double M, double r)
    {
        double omega = std::sqrt(G * M / std::pow(r, 3));
        return omega;
    }

    void update()
    {
        double angular_velocity_ = calculate_omega(g_constant_, mass_, orbit_radius_);
        double angle = elapsed_time_ * angular_velocity_;

        double x = orbit_radius_ * cos(angle);
        double y = orbit_radius_ * sin(angle);

        elapsed_time_ += 0.01;

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now();
        t.header.frame_id = "sun";
        t.child_frame_id = name_;
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = 0.0;
        t.transform.rotation.w = 1.0;

        // Broadcast the transform.
        broadcaster_.sendTransform(t);

        // Update the marker.
        marker_.header.stamp = now();
        marker_.pose.position.x = x;
        marker_.pose.position.y = y;
        marker_.pose.position.z = 0.0;
        marker_.pose.orientation.w = 1.0;

        vis_pub->publish(marker_);
    }

    std::string name_;
    double mass_;
    std::vector<double> position_;
    std::vector<double> velocity_;
    double g_constant_;
    double orbit_radius_;
    rclcpp::TimerBase::SharedPtr timer_;
    double elapsed_time_ = 0.0;
    tf2_ros::TransformBroadcaster broadcaster_;
    visualization_msgs::msg::Marker marker_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub;
};