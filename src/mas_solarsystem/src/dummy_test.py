import rclpy
from rclpy.node import Node
import math

class CelestialBody(Node):
    def __init__(self, name, mass, position, velocity, g_constant=6.67430e-11,publish_topic="position"):
        super().__init__(name)
        self.mass = mass
        self.position = position
        self.velocity = velocity
        self.g_constant = g_constant
        self.create_timer(0.1, self.move)  # Execute `move` method every 0.1 seconds
        # create a publisher for the position
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            publish_topic,
            10
        )

    def move(self):
        omega = math.sqrt(self.g_constant * self.mass / pow(self.distance(), 3))
        self.position[0] += self.velocity[0] * omega
        self.position[1] += self.velocity[1] * omega
        self.position[2] += self.velocity[2] * omega

        self.get_logger().info(f"{self.get_name()} position: {self.position}")

        # publish the position
        msg = Float64MultiArray()
        msg.data = self.position
        self.publisher_.publish(msg)
        
    def distance(self):
        # Calculate the distance from the origin (0,0,0) to the given position
        return math.sqrt(pow(self.position[0], 2) + pow(self.position[1], 2) + pow(self.position[2], 2))
