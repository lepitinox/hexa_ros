#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "celestial_body.h"
#include <string>



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::string name = argv[1];
    double mass = std::stod(argv[2]);
    double orbit_radius = std::stod(argv[3]);

    auto planet = std::make_shared<CelestialBody>(name, mass, orbit_radius);

    rclcpp::spin(planet);  // Keep the script running with earth as the main node

    // Destroy the nodes when the script is stopped
    rclcpp::shutdown();

    return 0;
}

