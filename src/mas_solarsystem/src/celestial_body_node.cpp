#include <iostream>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // log args
    for (int i = 0; i < argc; i++) {
        std::cout << argv[i] << std::endl;
    }

    // log args using rclcpp
    auto logger = rclcpp::get_logger("celestial_body_node");
    std::stringstream ss;
    for (int i = 1; i < argc; i++) {
      ss << argv[i] << ' ';
    }
    RCLCPP_INFO(logger, "celestial_body_node started with args: %s", ss.str().c_str());
        
    // use parameters from launch file
    auto planet = std::make_shared<CelestialBody>("planet", 5.972e24, Vector3d(0, 147e9, 0), Vector3d(30e3, 0, 0));

//    auto earth = std::make_shared<CelestialBody>("Earth", 5.972e24, Vector3d(0, 147e9, 0), Vector3d(30e3, 0, 0));
//    auto moon = std::make_shared<CelestialBody>("Moon", 7.342e22, Vector3d(385e6, 147e9, 0), Vector3d(1e3, 30e3, 0));
    rclcpp::spin(planet);  // Keep the script running with earth as the main node

    // Destroy the nodes when the script is stopped
    rclcpp::shutdown();

    return 0;
}

