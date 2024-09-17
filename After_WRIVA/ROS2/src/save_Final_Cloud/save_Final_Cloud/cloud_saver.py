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



class Cloud_Saver_Subscriber(Node):
    def __init__(self):
        super().__init__('cloud_saver')
        self.PC2_Save = None 

        self.Save_subscription = self.create_subscription(
            PointCloud2,
            'laser_cloud_surround',
            self.save_callback,
            10)
        self.Save_subscription

        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.jsonsave_callback)

    def save_callback(self, msg): 
        self.get_logger().info("Cloud recieved, resetting timer")
        self.destroy_timer(self.timer)
        self.PC2_Save = msg
        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.jsonsave_callback)


    def jsonsave_callback(self, msg="(:"):
        self.PC2_Save = message_converter.convert_ros_message_to_dictionary(self.PC2_Save, False)
        # del self.PC2_Save['topic']
        # del self.PC2_Save['time_ns']
        # del self.PC2_Save['type']
        with open('/home/rfal/test/final_PC2_as_dictionaryJson-NoLockStep.json', 'w') as json_file:
            json.dump(self.PC2_Save, json_file, indent=4)  # `indent=4` makes the JSON file more readable
        self.get_logger().info("Json saved")
        sys.exit(1)


def main(args=None):
    # reader.set_filter(['/velodyne_points'])
    rclpy.init(args=args)

    cloud_saver = Cloud_Saver_Subscriber()

    rclpy.spin(cloud_saver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cloud_saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
