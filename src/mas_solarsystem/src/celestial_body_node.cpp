#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "celestial_body.h"



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto planet = std::make_shared<CelestialBody>(argv[1], argv[7],argv[2]);

    rclcpp::spin(planet);  // Keep the script running with earth as the main node

    // Destroy the nodes when the script is stopped
    rclcpp::shutdown();

    return 0;
}

