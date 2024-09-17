from full_run import imu2csv
import os,sys
import subprocess
from datetime import datetime
from dir_crawler import dir_crawler as dc
from rpy_opk.get_opk import rpy2opk_write
import color_scheme as clr
import csv
import argparse
import rosbag
import pdb
# import tf.transformations
from transforms3d.euler import quat2euler


DEFAULT_OUTPUT = "./results.csv"
DEFUALT_RPY_BASE_DIR = "./lego-loam-solution"
DEFAULT_OPK_BASE_DIR = "./opk-solution"
DEFAULT_GPS_FIELDS = ["Measurement_DateTime", "GPS_lat", "GPS_lon", "GPS_alt"]
DEFAULT_RPY_FIELDS = ["t", "roll", "pitch", "yaw"]
DEFAULT_RAW_IMU_DIR = "./raw-imu-solution"
TIME_TOLERANCE = 1.0

bag_path = [r"D:\2024-06-05\12-22-42\data_0.bag"]

def generate_filename(base_dir):
    # Make a novel filename based on the current datetime
    dt = datetime.now()
    run_idx = dt.strftime("%Y_%m_%d_%H_%M_%S_%f")
    this_file = os.path.join(base_dir, "{}.csv".format(run_idx)) # make the output csv
    return os.path.abspath(this_file) # Use abspath just to be sure

def imu2csv(bag_paths):
    
    # Check if output directory exists
    # if not os.path.isdir(DEFAULT_RAW_IMU_DIR):
    #     os.makedirs(DEFAULT_RAW_IMU_DIR)    
    
    # Make an output file
    #outpath = generate_filename(DEFAULT_RAW_IMU_DIR)
    outpath = "IMU2CSV_quaternion_still.csv"
    # outpath = generate_filename("DEFAULT.csv")
    with open(outpath, 'w') as outfilehandle:
        outfilehandle.write("t,roll,pitch,yaw\n")
        # s_ is for summed, i.e. summed roll
        for current_path in bag_paths:
            with rosbag.Bag(current_path, 'r') as open_bag:
                for topic, msg, t in open_bag.read_messages("/imu/data"):
                    orientation_q = msg.orientation
                    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                    
                    (s_roll, s_pitch, s_yaw) = quat2euler(orientation_list)

                    t = t.to_sec()

                    # s_roll = (s_roll + msg.angular_velocity.x) % 360
                    # s_pitch = (s_pitch + msg.angular_velocity.y) % 360
                    # s_yaw = (s_yaw + msg.angular_velocity.z) % 360
                    outfilehandle.write("{},{},{},{}\n".format(
                        t,
                        s_roll,
                        s_pitch,
                        s_yaw
                    ))


    return [outpath] # Formatted as a list to play nice with downstream stuff

imu2csv(bag_path)