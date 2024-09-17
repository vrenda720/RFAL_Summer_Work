import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
# from sensor_msgs import point_cloud2
from sensor_msgs.msg import Imu
from std_msgs.msg import String
# from pprint import pprint
import nml_bag
from os import sep
# import tempfile
from pprint import pprint
# import rosbag2_py
import rclpy.clock
from example_interfaces import msg
from rclpy.serialization import serialize_message
# from nml_bag import Reader
# from ros2_message_converter import message_converter
from rclpy_message_converter import message_converter
import time
# import ros_numpy
import sensor_msgs_py.point_cloud2 as pc2
import json
import sys

class spammer(Node):
    def __init__(self):
        super().__init__('lock_step_node')
        # self.file = open('/home/rfal/test/final_PC2_as_dictionaryJson.json')
        self.file = open('/home/rfal/test/final_PC2_as_dictionaryJson.json')
        self.file = json.load(self.file)
        # del self.file['topic']
        # del self.file['time_ns']
        # del self.file['type']
        self.message = message_converter.convert_dictionary_to_ros_message("sensor_msgs/PointCloud2", self.file)
        self.message.header.stamp = self.get_clock().now().to_msg()
        
        self.publisher_cloud = self.create_publisher(
            PointCloud2, 
            'laser_cloud_surround', 
            10)

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.send_cloud_callback)

    def send_cloud_callback(self): 
        self.publisher_cloud.publish(self.message)
        self.get_logger().info("PC2 sent")


    

def main(args=None):
    rclpy.init(args=args)

    spamming = spammer()

    rclpy.spin(spamming)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spamming.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
