#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "celestial_body.h"
#include <string>



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CelestialBody>());  // Keep the script running with earth as the main node
    // Destroy the nodes when the script is stopped
    rclcpp::shutdown();
    return 0;
}

