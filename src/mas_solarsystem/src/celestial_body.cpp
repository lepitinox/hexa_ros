#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>
#include <memory>
#include <vector>

class CelestialBody : public rclcpp::Node
{
public:
    CelestialBody(const std::string &name, double mass, std::vector<double> position, std::vector<double> velocity, double g_constant = 6.67430e-11, const std::string &publish_topic = "position")
        : Node(name), mass_(mass), position_(position), velocity_(velocity), g_constant_(g_constant)
    {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CelestialBody::move, this));
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(publish_topic, 10);
    }

private:
    void move()
    {
        double omega = std::sqrt(g_constant_ * mass_ / std::pow(distance(), 3));
        position_[0] += velocity_[0] * omega;
        position_[1] += velocity_[1] * omega;
        position_[2] += velocity_[2] * omega;

        RCLCPP_INFO(this->get_logger(), "%s position: [%f, %f, %f]", this->get_name(), position_[0], position_[1], position_[2]);

        std_msgs::msg::Float64MultiArray msg;
        msg.data = position_;
        publisher_->publish(msg);
    }

    double distance()
    {
        return std::sqrt(std::pow(position_[0], 2) + std::pow(position_[1], 2) + std::pow(position_[2], 2));
    }

    double mass_;
    std::vector<double> position_;
    std::vector<double> velocity_;
    double g_constant_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto celestial_body = std::make_shared<CelestialBody>("name", mass, position, velocity);
    rclcpp::spin(celestial_body);
    rclcpp::shutdown();
    return 0;
}