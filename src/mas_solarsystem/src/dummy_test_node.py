from dummy_test import CelestialBody
import rclpy


def main(args=None):
    rclpy.init(args=args)

    # log args
    print(args)
    # log args using rclpy
    rclpy.logging.get_logger("dummy_test_node").info(args)

#    earth = CelestialBody("Earth", 5.972e24, [0, 147e9, 0], [30e3, 0, 0])
#    moon = CelestialBody("Moon", 7.342e22, [385e6, 147e9, 0], [1e3, 30e3, 0])

#    rclpy.spin(earth)  # Keep the script running with earth as the main node

    # Destroy the nodes when the script is stopped
    moon.destroy_node()
    earth.destroy_node()
    rclpy.shutdown()