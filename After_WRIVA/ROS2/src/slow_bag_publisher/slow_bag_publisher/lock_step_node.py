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

class Lock_step_publisher(Node):
    def __init__(self, reader):
        super().__init__('lock_step_node')
        self.reader = reader
        self.PC2_Save = None 

        self.publisher_velodyne = self.create_publisher(
            PointCloud2, 
            'velodyne_points', 
            10)
        self.publisher_imu = self.create_publisher(
            Imu, 
            'imu/data', 
            10)
        
        self.subscription = self.create_subscription(
            PointCloud2,
            'outlier_cloud', # 'registered_cloud', # 'laser_cloud_surround',
            self.next_step_callback,
            10)
        self.subscription

        self.Save_subscription = self.create_subscription(
            PointCloud2,
            'laser_cloud_surround',
            self.save_callback,
            10)
        self.Save_subscription

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.next_step_callback)

    def save_callback(self, msg): 
        self.PC2_Save = msg


    def next_step_callback(self, msg="(:"):
        self.destroy_timer(self.timer)
        try:
            PC2_message = self.reader.__next__()
        except StopIteration:
             
            self.PC2_Save = message_converter.convert_ros_message_to_dictionary(self.PC2_Save, False)
            with open('/home/rfal/test/final_PC2_as_dictionaryJson.json', 'w') as json_file:
                json.dump(self.PC2_Save, json_file, indent=4)  # `indent=4` makes the JSON file more readable
            sys.exit(1)

        # del message['topic']
        # del message['time_ns']
        # del message['type']

        # self.get_logger().info(type(message))

        # message = message_converter.convert_dictionary_to_ros_message("sensor_msgs/PointCloud2", message)
        # self.publisher_velodyne.publish(message)

        # self.publisher_imu.publish()
        # self.get_logger().info('Publishing')
        # pprint(self.reader.__next__()['type'])
        # self.get_logger().info('I heard: "%s"' % msg.data)
        # self.get_logger().info(message.keys())
        while(PC2_message['type'] != 'sensor_msgs/msg/PointCloud2'):
        # while True:
            # if (PC2_message['type'] != 'sensor_msgs/msg/PointCloud2'):
            imu_message = PC2_message
            del imu_message['topic']
            del imu_message['time_ns']
            del imu_message['type']
            imu_message = message_converter.convert_dictionary_to_ros_message("sensor_msgs/Imu", imu_message)
            imu_message.header.stamp = self.get_clock().now().to_msg()
            self.publisher_imu.publish(imu_message)
            self.get_logger().info("Imu Sent")
            # PC2_message = self.reader.__next__()
            try:
                PC2_message = self.reader.__next__()
            except StopIteration:
                self.PC2_Save = message_converter.convert_ros_message_to_dictionary(self.PC2_Save, False)
                with open('/home/rfal/test/final_PC2_as_dictionaryJson.json', 'w') as json_file:
                    json.dump(self.PC2_Save, json_file, indent=4)  # `indent=4` makes the JSON file more readable
                sys.exit(1)
            # else:
            #     del PC2_message['topic']
            #     del PC2_message['time_ns']
            #     del PC2_message['type']

            #     PC2_message = message_converter.convert_dictionary_to_ros_message("sensor_msgs/PointCloud2", PC2_message)
            #     PC2_message.header.stamp = self.get_clock().now().to_msg()
            #     self.publisher_velodyne.publish(PC2_message)
            #     self.get_logger().info("PC2 sent")
            #     PC2_message = self.reader.__next__()
            # time.sleep(1)
        
        # imu_message = self.reader.__next__()

        # self.get_logger().info(PC2_message['type'])
        # self.get_logger().info(imu_message['type'])
        del PC2_message['topic']
        del PC2_message['time_ns']
        del PC2_message['type']
        # del imu_message['topic']
        # del imu_message['time_ns']
        # del imu_message['type']

        PC2_message = message_converter.convert_dictionary_to_ros_message("sensor_msgs/PointCloud2", PC2_message)
        # imu_message = message_converter.convert_dictionary_to_ros_message("sensor_msgs/Imu", imu_message)
        PC2_message.header.stamp = self.get_clock().now().to_msg()
        # self.get_logger().info(imu_message)
        # imu_message.header
        # PC2_message['header']
        # pcnumpy = pc2.read_points(PC2_message)

        self.publisher_velodyne.publish(PC2_message)
        self.get_logger().info("PC2 sent")
        # self.publisher_imu.publish(imu_message)
        # self.get_logger().info("Imu Sent")
        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.next_step_callback)

def main(args=None):
    reader = nml_bag.Reader('/home/rfal/ros2_ws/bag_files/legoloam_dataset/bags/fullBag/fullBag_0.db3', storage_id='sqlite3')
    reader.set_filter(['/imu/data','/velodyne_points'])
    # reader.set_filter(['/velodyne_points'])
    rclpy.init(args=args)

    lock_step_publisher = Lock_step_publisher(reader)

    rclpy.spin(lock_step_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lock_step_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
