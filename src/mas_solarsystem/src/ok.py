import math
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
import rclpy
from rclpy.node import Node

class CelestialBody(Node):
    def __init__(self, name, orbit, radius, period, rotation, inclination, mass):
        super().__init__('celestial_body')
        self.orbit = orbit
        self.radius = radius
        self.period = period
        self.rotation = rotation
        self.inclination = inclination
        self.mass = mass
        self.name = name
        self.tf_broadcaster = TransformBroadcaster(self)
        self.broadcaster_timer = self.create_timer(0.05, self.publish_transform)

    def publish_transform(self):
        self.get_logger().info('Publishing celestial body position...')
        
        t = self.get_clock().now().seconds_nanoseconds()
        angle = 2 * math.pi * t[0] / self.period

        x = self.orbit * math.cos(angle)
        y = self.orbit * math.sin(angle)
        z = self.orbit * math.sin(self.inclination)
        
        qx = Quaternion()
        qx.x, qx.y, qx.z, qx.w = math.sin(self.rotation / 2), 0, 0, math.cos(self.rotation / 2)
        self.tf_broadcaster.sendTransform(self.euler_to_quaternion(angle, self.inclination, 0), (x, y, z), msg.frame_id, msg.child_frame_id)

    def euler_to_quaternion(self, roll, pitch, yaw):
        return Quaternion(x=roll, y=pitch, z=yaw)

def main(args=None):
    rclpy.init(args=args)
    celestial_body = CelestialBody(orbit=1.496e11, radius=6.9634e8, period=365.25*24*60*60, rotation=2*math.pi/24, inclination=0, mass=1.989e30)
    rclpy.spin(celestial_body)
    celestial_body.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()