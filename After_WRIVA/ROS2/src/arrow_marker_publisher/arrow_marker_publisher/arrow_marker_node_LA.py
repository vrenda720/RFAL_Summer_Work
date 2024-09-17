import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
from sensor_msgs.msg import Imu


class ArrowMarkerPublisher(Node):
    def __init__(self):
        super().__init__('arrow_marker_publisher_LA')
        self.publisher_LA = self.create_publisher(Marker, 'arrow_marker_Linear_Acceleration', 10)
        # self.timer = self.create_timer(1.0, self.timer_callback)
        self.getImu_subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.send_arrows_callback,
            10)
        self.getImu_subscription

        # Initialize the marker
        self.marker_LA = Marker()
        self.marker_LA.header.frame_id = "base_link"
        self.marker_LA.header.stamp = self.get_clock().now().to_msg()
        self.marker_LA.ns = "arrow_marker"
        self.marker_LA.id = 0
        self.marker_LA.type = Marker.ARROW
        self.marker_LA.action = Marker.ADD

        # Set the scale of the arrow
        self.marker_LA.scale.x = 0.05  # Shaft diameter
        self.marker_LA.scale.y = 0.2  # Head diameter
        self.marker_LA.scale.z = 0.5   # Length of the arrow

        # Set the color of the marker
        self.marker_LA.color.r = 1.0  # Red
        self.marker_LA.color.g = 0.0  # Green
        self.marker_LA.color.b = 0.0  # Blue
        self.marker_LA.color.a = 1.0   # Alpha (1.0 is fully opaque)

        
    def send_arrows_callback(self, msg):
        end_x = msg.linear_acceleration.x
        end_y = msg.linear_acceleration.y
        end_z = msg.linear_acceleration.z #+ 9.81  Figure out how to correctly get rid of gravity when the frame rotates
        start_point = Point(x=0.0, y=0.0, z=0.0)  # Starting position
        end_point = Point(x=end_x, y=end_y, z=end_z)
        self.marker_LA.points = [start_point, end_point]
        self.marker_LA.header.stamp = self.get_clock().now().to_msg()
        self.publisher_LA.publish(self.marker_LA)
        
def main(args=None):
    rclpy.init(args=args)
    arrow_marker_publisher = ArrowMarkerPublisher()
    rclpy.spin(arrow_marker_publisher)

    # Destroy the node explicitly
    arrow_marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
