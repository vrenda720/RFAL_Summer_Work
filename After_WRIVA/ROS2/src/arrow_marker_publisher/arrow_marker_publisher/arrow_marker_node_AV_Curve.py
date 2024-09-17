import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
from sensor_msgs.msg import Imu
import numpy as np
import math

def arrow_direction_point(point_a, point_b, distance_from_a):
    direction_x = point_b.x - point_a.x
    direction_y = point_b.y - point_a.y
    direction_z = point_b.z - point_a.z
    magnitude = np.sqrt((direction_x ** 2)+(direction_y ** 2)+(direction_z ** 2))
    normal_x = direction_x / magnitude
    normal_y = direction_y / magnitude
    normal_z = direction_z / magnitude
    scaled_vect_x = distance_from_a * normal_x
    scaled_vect_y = distance_from_a * normal_y
    scaled_vect_z = distance_from_a * normal_z
    return Point(x=scaled_vect_x + point_a.x, y=scaled_vect_y + point_a.y, z=scaled_vect_z + point_a.z)


def spherical_to_cartesian(r, theta, phi):
    """Convert spherical coordinates to Cartesian coordinates."""
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta)
    return np.array([x, y, z])

def cartesian_to_spherical(x, y, z):
    """Convert Cartesian coordinates back to spherical coordinates."""
    r = np.sqrt(x**2 + y**2 + z**2)
    theta = np.arccos(z / r) if r != 0 else 0
    phi = np.arctan2(y, x)
    return r, theta, phi

def move_point_on_sphere(r, theta, phi, distance, direction, ref_point):
    """Move a point on the surface of a sphere."""
    # Convert to Cartesian coordinates
    x, y, z = spherical_to_cartesian(r, theta, phi)

    # Convert the movement distance and direction to Cartesian coordinates
    # The movement is in the direction defined by the azimuthal angle (phi)
    dx = distance * np.cos(direction)
    dy = distance * np.sin(direction)
    
    # Move the point in Cartesian coordinates
    # new_x = x + dx
    # new_y = y
    # new_z = z + dy # We keep the z-coordinate unchanged for horizontal movement

    if ref_point == "top":
        new_x = x + dx
        new_y = y + dy
        new_z = z
    elif ref_point == "right":
        new_x = x + dx
        new_y = y 
        new_z = z + dy
    elif ref_point == "left":
        new_x = x - dx
        new_y = y 
        new_z = z - dy
    elif ref_point == "front":
        new_x = x
        new_y = y + dx
        new_z = z + dy
    elif ref_point == "back":
        new_x = x
        new_y = y - dx
        new_z = z - dy

    # Normalize the new point to stay on the surface of the sphere
    new_position = np.array([new_x, new_y, new_z])
    new_position_normalized = new_position / np.linalg.norm(new_position) * r
    
    # Convert back to spherical coordinates
    return cartesian_to_spherical(*new_position_normalized)


class ArrowMarkerPublisher(Node):
    def __init__(self):
        super().__init__('arrow_marker_publisher_AV_curve')

        self.publisher_Curve_Body_Right = self.create_publisher(Marker, 'Curve_Body_Right_marker', 10)
        self.publisher_Curve_Body_Left = self.create_publisher(Marker, 'Curve_Body_Left_marker', 10)
        self.publisher_Curve_Head_Right = self.create_publisher(Marker, 'Curve_Head_Right_marker', 10)
        self.publisher_Curve_Head_Left = self.create_publisher(Marker, 'Curve_Head_Left_marker', 10)

        self.publisher_Curve_Body_Front = self.create_publisher(Marker, 'Curve_Body_Front_marker', 10)
        self.publisher_Curve_Head_Front = self.create_publisher(Marker, 'Curve_Head_Front_marker', 10)
        self.publisher_Curve_Body_Back = self.create_publisher(Marker, 'Curve_Body_Back_marker', 10)
        self.publisher_Curve_Head_Back = self.create_publisher(Marker, 'Curve_Head_Back_marker', 10)
        # self.timer = self.create_timer(1.0, self.timer_callback)
        self.getImu_subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.send_arrows_callback,
            10)
        self.getImu_subscription

        # Initialize the marker

########## Angular Velocity Curve Body Right
        self.marker_Curve_Body_AV_R = Marker()
        self.marker_Curve_Body_AV_R.header.frame_id = "base_link"
        self.marker_Curve_Body_AV_R.header.stamp = self.get_clock().now().to_msg()
        self.marker_Curve_Body_AV_R.ns = "line_strip"
        self.marker_Curve_Body_AV_R.id = 0
        self.marker_Curve_Body_AV_R.type = Marker.LINE_STRIP
        self.marker_Curve_Body_AV_R.action = Marker.ADD

        # Set the scale of the arrow
        self.marker_Curve_Body_AV_R.scale.x = 0.05  # Shaft diameter
        # self.marker_Curve_Body_AV_R.scale.y = 0.5  # Head diameter
        # self.marker_Curve_Body_AV_R.scale.z = 0.5   # Length of the arrow

        # Set the color of the marker
        self.marker_Curve_Body_AV_R.color.r = 0.0  # Red
        self.marker_Curve_Body_AV_R.color.g = 0.0  # Green
        self.marker_Curve_Body_AV_R.color.b = 1.0  # Blue
        self.marker_Curve_Body_AV_R.color.a = 1.0   # Alpha (1.0 is fully opaque)


########## Angular Velocity Curve Body Left
        self.marker_Curve_Body_AV_L = Marker()
        self.marker_Curve_Body_AV_L.header.frame_id = "base_link"
        self.marker_Curve_Body_AV_L.header.stamp = self.get_clock().now().to_msg()
        self.marker_Curve_Body_AV_L.ns = "line_strip"
        self.marker_Curve_Body_AV_L.id = 0
        self.marker_Curve_Body_AV_L.type = Marker.LINE_STRIP
        self.marker_Curve_Body_AV_L.action = Marker.ADD

        # Set the scale of the arrow
        self.marker_Curve_Body_AV_L.scale.x = 0.05  # Shaft diameter
        # self.marker_Curve_Body_AV_L.scale.y = 0.5  # Head diameter
        # self.marker_Curve_Body_AV_L.scale.z = 0.5   # Length of the arrow

        # Set the color of the marker
        self.marker_Curve_Body_AV_L.color.r = 0.0  # Red
        self.marker_Curve_Body_AV_L.color.g = 0.0  # Green
        self.marker_Curve_Body_AV_L.color.b = 1.0  # Blue
        self.marker_Curve_Body_AV_L.color.a = 1.0   # Alpha (1.0 is fully opaque)

########## Angular Velocity Curve Head Right
        self.marker_Curve_Head_AV_R = Marker()
        self.marker_Curve_Head_AV_R.header.frame_id = "base_link"
        self.marker_Curve_Head_AV_R.header.stamp = self.get_clock().now().to_msg()
        self.marker_Curve_Head_AV_R.ns = "arrow_marker"
        self.marker_Curve_Head_AV_R.id = 0
        self.marker_Curve_Head_AV_R.type = Marker.ARROW
        self.marker_Curve_Head_AV_R.action = Marker.ADD

        # Set the scale of the arrow
        self.marker_Curve_Head_AV_R.scale.x = 0.0  # Shaft diameter
        self.marker_Curve_Head_AV_R.scale.y = 0.2  # Head diameter
        self.marker_Curve_Head_AV_R.scale.z = 0.5   # Length of the arrow

        # Set the color of the marker
        self.marker_Curve_Head_AV_R.color.r = 0.0  # Red
        self.marker_Curve_Head_AV_R.color.g = 0.0  # Green
        self.marker_Curve_Head_AV_R.color.b = 1.0  # Blue
        self.marker_Curve_Head_AV_R.color.a = 1.0   # Alpha (1.0 is fully opaque)

########## Angular Velocity Curve Head Left
        self.marker_Curve_Head_AV_L = Marker()
        self.marker_Curve_Head_AV_L.header.frame_id = "base_link"
        self.marker_Curve_Head_AV_L.header.stamp = self.get_clock().now().to_msg()
        self.marker_Curve_Head_AV_L.ns = "arrow_marker"
        self.marker_Curve_Head_AV_L.id = 0
        self.marker_Curve_Head_AV_L.type = Marker.ARROW
        self.marker_Curve_Head_AV_L.action = Marker.ADD

        # Set the scale of the arrow
        self.marker_Curve_Head_AV_L.scale.x = 0.0  # Shaft diameter
        self.marker_Curve_Head_AV_L.scale.y = 0.2  # Head diameter
        self.marker_Curve_Head_AV_L.scale.z = 0.5   # Length of the arrow

        # Set the color of the marker
        self.marker_Curve_Head_AV_L.color.r = 0.0  # Red
        self.marker_Curve_Head_AV_L.color.g = 0.0  # Green
        self.marker_Curve_Head_AV_L.color.b = 1.0  # Blue
        self.marker_Curve_Head_AV_L.color.a = 1.0   # Alpha (1.0 is fully opaque)

########## Angular Velocity Curve Body Front
        self.marker_Curve_Body_AV_F = Marker()
        self.marker_Curve_Body_AV_F.header.frame_id = "base_link"
        self.marker_Curve_Body_AV_F.header.stamp = self.get_clock().now().to_msg()
        self.marker_Curve_Body_AV_F.ns = "line_strip"
        self.marker_Curve_Body_AV_F.id = 0
        self.marker_Curve_Body_AV_F.type = Marker.LINE_STRIP
        self.marker_Curve_Body_AV_F.action = Marker.ADD

        # Set the scale of the arrow
        self.marker_Curve_Body_AV_F.scale.x = 0.05  # Shaft diameter
        # self.marker_Curve_Body_AV_R.scale.y = 0.5  # Head diameter
        # self.marker_Curve_Body_AV_R.scale.z = 0.5   # Length of the arrow

        # Set the color of the marker
        self.marker_Curve_Body_AV_F.color.r = 0.0  # Red
        self.marker_Curve_Body_AV_F.color.g = 0.0  # Green
        self.marker_Curve_Body_AV_F.color.b = 1.0  # Blue
        self.marker_Curve_Body_AV_F.color.a = 1.0   # Alpha (1.0 is fully opaque)

########## Angular Velocity Curve Head Front
        self.marker_Curve_Head_AV_F = Marker()
        self.marker_Curve_Head_AV_F.header.frame_id = "base_link"
        self.marker_Curve_Head_AV_F.header.stamp = self.get_clock().now().to_msg()
        self.marker_Curve_Head_AV_F.ns = "arrow_marker"
        self.marker_Curve_Head_AV_F.id = 0
        self.marker_Curve_Head_AV_F.type = Marker.ARROW
        self.marker_Curve_Head_AV_F.action = Marker.ADD

        # Set the scale of the arrow
        self.marker_Curve_Head_AV_F.scale.x = 0.0  # Shaft diameter
        self.marker_Curve_Head_AV_F.scale.y = 0.2  # Head diameter
        self.marker_Curve_Head_AV_F.scale.z = 0.5   # Length of the arrow

        # Set the color of the marker
        self.marker_Curve_Head_AV_F.color.r = 0.0  # Red
        self.marker_Curve_Head_AV_F.color.g = 0.0  # Green
        self.marker_Curve_Head_AV_F.color.b = 1.0  # Blue
        self.marker_Curve_Head_AV_F.color.a = 1.0   # Alpha (1.0 is fully opaque)


########## Angular Velocity Curve Body Back
        self.marker_Curve_Body_AV_B = Marker()
        self.marker_Curve_Body_AV_B.header.frame_id = "base_link"
        self.marker_Curve_Body_AV_B.header.stamp = self.get_clock().now().to_msg()
        self.marker_Curve_Body_AV_B.ns = "line_strip"
        self.marker_Curve_Body_AV_B.id = 0
        self.marker_Curve_Body_AV_B.type = Marker.LINE_STRIP
        self.marker_Curve_Body_AV_B.action = Marker.ADD

        # Set the scale of the arrow
        self.marker_Curve_Body_AV_B.scale.x = 0.05  # Shaft diameter
        # self.marker_Curve_Body_AV_R.scale.y = 0.5  # Head diameter
        # self.marker_Curve_Body_AV_R.scale.z = 0.5   # Length of the arrow

        # Set the color of the marker
        self.marker_Curve_Body_AV_B.color.r = 0.0  # Red
        self.marker_Curve_Body_AV_B.color.g = 0.0  # Green
        self.marker_Curve_Body_AV_B.color.b = 1.0  # Blue
        self.marker_Curve_Body_AV_B.color.a = 1.0   # Alpha (1.0 is fully opaque)

########## Angular Velocity Curve Head Back
        self.marker_Curve_Head_AV_B = Marker()
        self.marker_Curve_Head_AV_B.header.frame_id = "base_link"
        self.marker_Curve_Head_AV_B.header.stamp = self.get_clock().now().to_msg()
        self.marker_Curve_Head_AV_B.ns = "arrow_marker"
        self.marker_Curve_Head_AV_B.id = 0
        self.marker_Curve_Head_AV_B.type = Marker.ARROW
        self.marker_Curve_Head_AV_B.action = Marker.ADD

        # Set the scale of the arrow
        self.marker_Curve_Head_AV_B.scale.x = 0.0  # Shaft diameter
        self.marker_Curve_Head_AV_B.scale.y = 0.2  # Head diameter
        self.marker_Curve_Head_AV_B.scale.z = 0.5   # Length of the arrow

        # Set the color of the marker
        self.marker_Curve_Head_AV_B.color.r = 0.0  # Red
        self.marker_Curve_Head_AV_B.color.g = 0.0  # Green
        self.marker_Curve_Head_AV_B.color.b = 1.0  # Blue
        self.marker_Curve_Head_AV_B.color.a = 1.0   # Alpha (1.0 is fully opaque)



    def send_arrows_callback(self, msg):
        ####### AV Right Head+Body Curve
        x1 = 0.0
        y1 = -1.0
        z1 = 0.0
        
        points = [Point(x=x1, y=y1, z=z1)]
        for num in range(1, 9):
            magnitude = np.sqrt((msg.angular_velocity.x ** 2) + (msg.angular_velocity.z ** 2))
            distance = (magnitude ** 2) * ((2 ** num)/4)
            direction = np.arctan2(msg.angular_velocity.x, msg.angular_velocity.z)
            r, theta, phi = cartesian_to_spherical(x1, y1, z1)
            new_r, new_theta, new_phi = move_point_on_sphere(r, theta, phi, distance, direction, ref_point="right")
            new_point = spherical_to_cartesian(new_r, new_theta, new_phi)
            points.append(Point(x=new_point[0], y=new_point[1], z=new_point[2]))

        self.marker_Curve_Body_AV_R.points = points[0:7]
        self.marker_Curve_Body_AV_R.header.stamp = self.get_clock().now().to_msg()
        self.publisher_Curve_Body_Right.publish(self.marker_Curve_Body_AV_R)
        
        start_point = points[6]  # Starting position
        if(points[6] == points[7]):
            end_point = points[6]
        else:
            end_point = arrow_direction_point(points[6], points[7], magnitude)
        self.marker_Curve_Head_AV_R.points = [start_point, end_point]
        self.marker_Curve_Head_AV_R.header.stamp = self.get_clock().now().to_msg()
        self.publisher_Curve_Head_Right.publish(self.marker_Curve_Head_AV_R)
        
        
        ####### AV Left Head+Body Curve
        x1 = 0.0
        y1 = 1.0
        z1 = 0.0
        
        points = [Point(x=x1, y=y1, z=z1)]
        for num in range(1, 9):
            magnitude = np.sqrt((msg.angular_velocity.x ** 2) + (msg.angular_velocity.z ** 2))
            distance = (magnitude ** 2) * ((2 ** num)/4)
            # magnitude = magnitude + " "
            # self.get_logger().info(magnitude + " ")
            direction = np.arctan2(msg.angular_velocity.x, msg.angular_velocity.z)
            r, theta, phi = cartesian_to_spherical(x1, y1, z1)
            new_r, new_theta, new_phi = move_point_on_sphere(r, theta, phi, distance, direction, ref_point="left")
            new_point = spherical_to_cartesian(new_r, new_theta, new_phi)
            points.append(Point(x=new_point[0], y=new_point[1], z=new_point[2]))

        self.marker_Curve_Body_AV_L.points = points[0:7]
        self.marker_Curve_Body_AV_L.header.stamp = self.get_clock().now().to_msg()
        self.publisher_Curve_Body_Left.publish(self.marker_Curve_Body_AV_L)
        
        start_point = points[6]  # Starting position
        if(points[6] == points[7]):
            end_point = points[6]
        else:
            end_point = arrow_direction_point(points[6], points[7], magnitude)
        self.marker_Curve_Head_AV_L.points = [start_point, end_point]
        self.marker_Curve_Head_AV_L.header.stamp = self.get_clock().now().to_msg()
        self.publisher_Curve_Head_Left.publish(self.marker_Curve_Head_AV_L)

        ####### AV Front Head+Body Curve
        x1 = 1.0
        y1 = 0.0
        z1 = 0.0
        
        points = [Point(x=x1, y=y1, z=z1)]
        for num in range(1, 9):
            magnitude = np.sqrt((msg.angular_velocity.y ** 2) + (msg.angular_velocity.z ** 2))
            distance = (magnitude ** 2) * ((2 ** num)/4)
            direction = np.arctan2(msg.angular_velocity.y, msg.angular_velocity.z)
            r, theta, phi = cartesian_to_spherical(x1, y1, z1)
            new_r, new_theta, new_phi = move_point_on_sphere(r, theta, phi, distance, direction, ref_point="front")
            new_point = spherical_to_cartesian(new_r, new_theta, new_phi)
            points.append(Point(x=new_point[0], y=new_point[1], z=new_point[2]))

        self.marker_Curve_Body_AV_F.points = points[0:7]
        self.marker_Curve_Body_AV_F.header.stamp = self.get_clock().now().to_msg()
        self.publisher_Curve_Body_Front.publish(self.marker_Curve_Body_AV_F)
        
        start_point = points[6]  # Starting position
        if(points[6] == points[7]):
            end_point = points[6]
        else:
            end_point = arrow_direction_point(points[6], points[7], magnitude)
        self.marker_Curve_Head_AV_F.points = [start_point, end_point]
        self.marker_Curve_Head_AV_F.header.stamp = self.get_clock().now().to_msg()
        self.publisher_Curve_Head_Front.publish(self.marker_Curve_Head_AV_F)

        ####### AV Back Head+Body Curve
        x1 = -1.0
        y1 = 0.0
        z1 = 0.0
        
        points = [Point(x=x1, y=y1, z=z1)]
        magnitude = np.sqrt((msg.angular_velocity.y ** 2) + (msg.angular_velocity.z ** 2))
        direction = np.arctan2(msg.angular_velocity.y, msg.angular_velocity.z)
        for num in range(1, 9):
            # magnitude = np.sqrt((msg.angular_velocity.y ** 2) + (msg.angular_velocity.z ** 2))
            distance = (magnitude ** 1.6) * ((2 ** num)/4)
            # direction = np.arctan2(msg.angular_velocity.y, msg.angular_velocity.z)
            r, theta, phi = cartesian_to_spherical(x1, y1, z1)
            new_r, new_theta, new_phi = move_point_on_sphere(r, theta, phi, distance, direction, ref_point="back")
            new_point = spherical_to_cartesian(new_r, new_theta, new_phi)
            points.append(Point(x=new_point[0], y=new_point[1], z=new_point[2]))

        self.marker_Curve_Body_AV_B.points = points[0:7]
        self.marker_Curve_Body_AV_B.header.stamp = self.get_clock().now().to_msg()
        self.publisher_Curve_Body_Back.publish(self.marker_Curve_Body_AV_B)
        
        start_point = points[6]  # Starting position
        if(points[6] == points[7]):
            end_point = points[6]
        else:
            end_point = arrow_direction_point(points[6], points[7], magnitude)
        self.marker_Curve_Head_AV_B.points = [start_point, end_point]
        self.marker_Curve_Head_AV_B.header.stamp = self.get_clock().now().to_msg()
        self.publisher_Curve_Head_Back.publish(self.marker_Curve_Head_AV_B)
        
        
def main(args=None):
    rclpy.init(args=args)
    arrow_marker_publisher = ArrowMarkerPublisher()
    rclpy.spin(arrow_marker_publisher)

    # Destroy the node explicitly
    arrow_marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
