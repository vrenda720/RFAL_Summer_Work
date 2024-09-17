import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
from sensor_msgs.msg import Imu

class ArrowMarkerPublisher(Node):
    def __init__(self):
        super().__init__('arrow_marker_publisher_AV')
        self.publisher_AV_R = self.create_publisher(Marker, 'arrow_marker_Angular_Velocity_Right', 10)
        self.publisher_AV_L = self.create_publisher(Marker, 'arrow_marker_Angular_Velocity_Left', 10)
        self.publisher_AV_F = self.create_publisher(Marker, 'arrow_marker_Angular_Velocity_Front', 10)
        self.publisher_AV_B = self.create_publisher(Marker, 'arrow_marker_Angular_Velocity_Back', 10)

        # self.timer = self.create_timer(1.0, self.timer_callback)
        self.getImu_subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.send_arrows_callback,
            10)
        self.getImu_subscription

        # Initialize the marker
######## Angular Velocity Straight Right Arrow
        self.marker_AV_R = Marker()
        self.marker_AV_R.header.frame_id = "base_link"
        self.marker_AV_R.header.stamp = self.get_clock().now().to_msg()
        self.marker_AV_R.ns = "arrow_marker"
        self.marker_AV_R.id = 0
        self.marker_AV_R.type = Marker.ARROW
        self.marker_AV_R.action = Marker.ADD

        # Set the scale of the arrow
        self.marker_AV_R.scale.x = 0.2  # Shaft diameter
        self.marker_AV_R.scale.y = 0.4  # Head diameter
        self.marker_AV_R.scale.z = 0.7   # Length of the arrow

        # Set the color of the marker
        self.marker_AV_R.color.r = 1.0  # Red
        self.marker_AV_R.color.g = 0.0  # Green
        self.marker_AV_R.color.b = 0.0  # Blue
        self.marker_AV_R.color.a = 1.0   # Alpha (1.0 is fully opaque)

########## Angular Velocity Straight Left Arrow
        # Initialize the marker
        self.marker_AV_L = Marker()
        self.marker_AV_L.header.frame_id = "base_link"
        self.marker_AV_L.header.stamp = self.get_clock().now().to_msg()
        self.marker_AV_L.ns = "arrow_marker"
        self.marker_AV_L.id = 0
        self.marker_AV_L.type = Marker.ARROW
        self.marker_AV_L.action = Marker.ADD

        # Set the scale of the arrow
        self.marker_AV_L.scale.x = 0.2  # Shaft diameter
        self.marker_AV_L.scale.y = 0.4  # Head diameter
        self.marker_AV_L.scale.z = 0.7   # Length of the arrow

        # Set the color of the marker
        self.marker_AV_L.color.r = 1.0  # Red
        self.marker_AV_L.color.g = 0.0  # Green
        self.marker_AV_L.color.b = 0.0  # Blue
        self.marker_AV_L.color.a = 1.0   # Alpha (1.0 is fully opaque)

########## Angular Velocity Straight Front Arrow
        self.marker_AV_F = Marker()
        self.marker_AV_F.header.frame_id = "base_link"
        self.marker_AV_F.header.stamp = self.get_clock().now().to_msg()
        self.marker_AV_F.ns = "arrow_marker"
        self.marker_AV_F.id = 0
        self.marker_AV_F.type = Marker.ARROW
        self.marker_AV_F.action = Marker.ADD

        # Set the scale of the arrow
        self.marker_AV_F.scale.x = 0.2  # Shaft diameter
        self.marker_AV_F.scale.y = 0.4  # Head diameter
        self.marker_AV_F.scale.z = 0.7   # Length of the arrow

        # Set the color of the marker
        self.marker_AV_F.color.r = 0.0  # Red
        self.marker_AV_F.color.g = 0.0  # Green
        self.marker_AV_F.color.b = 1.0  # Blue
        self.marker_AV_F.color.a = 1.0   # Alpha (1.0 is fully opaque)

########## Angular Velocity Straight Back Arrow
        self.marker_AV_B = Marker()
        self.marker_AV_B.header.frame_id = "base_link"
        self.marker_AV_B.header.stamp = self.get_clock().now().to_msg()
        self.marker_AV_B.ns = "arrow_marker"
        self.marker_AV_B.id = 0
        self.marker_AV_B.type = Marker.ARROW
        self.marker_AV_B.action = Marker.ADD

        # Set the scale of the arrow
        self.marker_AV_B.scale.x = 0.2  # Shaft diameter
        self.marker_AV_B.scale.y = 0.4  # Head diameter
        self.marker_AV_B.scale.z = 0.7   # Length of the arrow

        # Set the color of the marker
        self.marker_AV_B.color.r = 0.0  # Red
        self.marker_AV_B.color.g = 0.0  # Green
        self.marker_AV_B.color.b = 1.0  # Blue
        self.marker_AV_B.color.a = 1.0   # Alpha (1.0 is fully opaque)



    def send_arrows_callback(self, msg):
        
        ####### Straight AV Arrows F B L R
        end_x = msg.angular_velocity.x * 25
        end_y = msg.angular_velocity.y # Can't display pitch with left, right arrow
        end_z = msg.angular_velocity.z * 25
        start_point = Point(x=0.0, y=-1.0, z=0.0)  # Starting position
        end_point = Point(x=end_z, y=-1.0, z=end_x)
        self.marker_AV_R.points = [start_point, end_point]
        start_point = Point(x=0.0, y=1.0, z=0.0)
        end_point = Point(x=(end_z) * -1.0, y=1.0, z=(end_x) * -1.0)
        self.marker_AV_L.points = [start_point, end_point]
        self.marker_AV_R.header.stamp = self.get_clock().now().to_msg()
        self.marker_AV_L.header.stamp = self.get_clock().now().to_msg()
        self.publisher_AV_R.publish(self.marker_AV_R)
        # self.get_logger().info('Publishing arrow marker: AV_R')
        self.publisher_AV_L.publish(self.marker_AV_L)
        # self.get_logger().info('Publishing arrow marker: AV_L')

        end_x = msg.angular_velocity.x # Can't display roll with front, back arrow
        end_y = msg.angular_velocity.y * 25
        end_z = msg.angular_velocity.z * 25
        start_point = Point(x=1.0, y=0.0, z=0.0)  # Starting position
        end_point = Point(x=1.0, y=end_z, z=end_y)
        self.marker_AV_F.points = [start_point, end_point]
        start_point = Point(x=-1.0, y=0.0, z=0.0)
        end_point = Point(x=-1.0, y=(end_z) * -1.0, z=(end_y) * -1.0)
        self.marker_AV_B.points = [start_point, end_point]
        self.marker_AV_F.header.stamp = self.get_clock().now().to_msg()
        self.marker_AV_B.header.stamp = self.get_clock().now().to_msg()
        self.publisher_AV_F.publish(self.marker_AV_F)
        # self.get_logger().info('Publishing arrow marker: AV_F')
        self.publisher_AV_B.publish(self.marker_AV_B)
        # self.get_logger().info('Publishing arrow marker: AV_B')
        

def main(args=None):
    rclpy.init(args=args)
    arrow_marker_publisher = ArrowMarkerPublisher()
    rclpy.spin(arrow_marker_publisher)

    # Destroy the node explicitly
    arrow_marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
