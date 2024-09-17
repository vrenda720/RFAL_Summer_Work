### Script for generating camera angles omega, phi, and kappa from raw bag files
# Bag files must have /velodyne_points and /imu/data messages
# Output is one single csv with OPK angles indexed by timestamp

import os,sys
import subprocess
from datetime import datetime
from dir_crawler import dir_crawler as dc
from rpy_opk.get_opk import rpy2opk_write
import color_scheme as clr
import csv
import argparse
import math
import rosbag
from transforms3d.euler import quat2euler

import pdb

DEFAULT_OUTPUT = "./results.csv"
DEFUALT_RPY_BASE_DIR = "./lego-loam-solution"
DEFAULT_OPK_BASE_DIR = "./opk-solution"
DEFAULT_GPS_FIELDS = ["Measurement_DateTime", "GPS_lat", "GPS_lon", "GPS_alt"]
DEFAULT_RPY_FIELDS = ["t", "roll", "pitch", "yaw"]
DEFAULT_RAW_IMU_DIR = "./raw-imu-solution"
TIME_TOLERANCE = 1.0

def handle_input(path, extension, *args):
    ### Handle input ###
    # Returns list of paths to files and list of file dirs
    # If input is valid file, returns list of length 1
    # If input is a directory, returns all files of specified type recursively
    # If input is list, basically a passthough
    # args can be used to provide custom error messages
    
    if os.path.isdir(path):
        # Directory crawler, BFS to find files of extension type
        session = dc(
            path,
            "",
            "",
            extension,
            False
        )
        session.traverse()
        session.reorganize()
        targets = session.printout()
        target_dirs = session.get_dirs()
        del session
    elif os.path.isfile(path):
        targets = [path] # list of len 1 for formatting purposes
        target_dirs = [os.path.dirname(path)]
    elif type(path) == list:
        targets = path   # basically a passthough
        target_dirs = []
        try:
            for item in path:
                dirname = os.path.dirname(item)
                if dirname not in target_dirs:
                    target_dirs.append(dirname)
        except:
            pass
    else:
        print("{}Error{}: {} is not a valid file or directory".format(
            clr.ERROR, clr.RESET, path
        ))
        try:
            for extra_info in args:
                print(extra_info)
        except:
            pass
        sys.exit(-1)
    return targets, target_dirs

def handle_output(outfilename, *args):
    ### Handle output file name ###
    # Check to see if output file is formatted properly before using

    # See if output file was provided (if none was, input should be None)
    if not outfilename:
        generated_name = generate_filename(args[0]) # generate novel filename
        usr_input = input("{}Warning{}: No output filename specified, continue with generated name: {}? [y/n] ".format(
            clr.WARNING, clr.RESET, generated_name)) 
        if usr_input != "y":
            sys.exit(1)
        else:
            output_filename = generated_name
    elif outfilename:
        output_filename = outfilename
        print("Using output filename {}{}{}".format(
            clr.HIGHLIGHT, output_filename, clr.RESET
        ))
    else:
        # Should never be reached but throw error just in case
        print("{}Error{}: {} is not a valid entry")
        sys.exit(-1)

    # See if file has proper extension
    basename, ext = os.path.splitext(output_filename)
    if ext != ".csv":
        if input("{}Warning{}: provided output {}{}{} is not a csv file. Rename to {}.csv and continue? [y/n] ".format(
            clr.WARNING, clr.RESET, clr.HIGHLIGHT, output_filename, clr.RESET, basename
        )) == "y":
            output_filename = basename + ".csv"
        else:
            sys.exit(1)
    
    # See if file already exists, and also make paths if needed
    if os.path.isfile(output_filename):
        if input("{}Warning{}: {} already exists. Continue anyway? [y/n]".format(
            clr.WARNING, clr.RESET, output_filename
        )) != "y":
            sys.exit(1)
    # If file doesn't exist, make sure there is a valid path
    else:
        target_dir = os.path.dirname(output_filename)
        if not os.path.isdir(target_dir):
            if input("{}Warning{}: Provided path {} does not exist. Create dirs and continue? [y/n] ".format(
                clr.WARNING, clr.RESET, target_dir
            )) != "y":
                sys.exit(1)
            os.makedirs(target_dir)
    # Touch just to be sure file is created
    os.system("touch {}".format(output_filename))
    return output_filename

def generate_filename(base_dir):
    # Make a novel filename based on the current datetime
    dt = datetime.now()
    run_idx = dt.strftime("%Y_%m_%d_%H_%M_%S_%f")
    this_file = os.path.join(base_dir, "{}.csv".format(run_idx)) # make the output csv
    return os.path.abspath(this_file) # Use abspath just to be sure

### Get RPY Solution from LeGO-LOAM ###
def rpy_solution(bag_files, bag_dirs):

    RPY_BASE_DIR = None  # TODO: make this changeable later, maybe though args

    # If nothing provided, use base dirname
    if not RPY_BASE_DIR:
        RPY_BASE_DIR = DEFUALT_RPY_BASE_DIR

    # Make sure dir exists
    if not os.path.isdir(RPY_BASE_DIR):
        os.makedirs(RPY_BASE_DIR)

    session_files = []  # Log of all output files created, for input into opk solution
    for data_dir in bag_dirs:
        # Use current datetime as an index, good for debugging and ensures unique file name
        this_file = generate_filename(RPY_BASE_DIR)
        session_files.append(this_file)
        # Touch file, since LeGO_LOAM sometimes complains if the file doesn't already exist
        os.system("touch {}".format(this_file))
        # Part of LeGO-LOAM payload
        trajectory_file_name = "trajectory_file_name:={}".format(this_file)

        # Each run consists of multiple bag files
        # Open a new session of LeGo-LOAM for each run
        # Runs are determined by directory
        # Opens trajectory csv for writing
        lego_loam_proc = subprocess.Popen(["roslaunch", 
                                        "lego_loam", 
                                        "run.launch", 
                                        trajectory_file_name])

        # Open bag files for the current run
        for bag_file in bag_files:
            if os.path.dirname(bag_file) == data_dir:
                bag_proc = subprocess.Popen(["rosbag",
                                    "play",
                                    bag_file,
                                    "--clock",
                                    "--topic",
                                    "/velodyne_points",
                                    "/imu/data"])
                # Wait until bag has played out
                bag_proc.wait()
            else:
                # Current bag was not in current run, skip it
                continue
        # TODO: Take screenshot to see if solution is good, then exit
        # Close LeGO-LOAM session, closes trajectory csv
        lego_loam_proc.terminate()
    # Return list of files created during this session
    return session_files

def imu2csv(bag_paths, verbose):
    
    # Check if output directory exists
    if not os.path.isdir(DEFAULT_RAW_IMU_DIR):
        os.makedirs(DEFAULT_RAW_IMU_DIR)    
    
    # Make an output file
    outpath = generate_filename(DEFAULT_RAW_IMU_DIR)
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
                        math.degrees(s_roll),
                        math.degrees(s_pitch),
                        math.degrees(s_yaw)
                    ))


    return [outpath] # Formatted as a list to play nice with downstream stuff

# def imu2csv(bag_paths, verbose):

#         # Check if output directory exists
#     if not os.path.isdir(DEFAULT_RAW_IMU_DIR):
#         os.makedirs(DEFAULT_RAW_IMU_DIR)

#     # Make an output file
#     outpath = generate_filename(DEFAULT_RAW_IMU_DIR)
#     with open(outpath, 'w') as outfilehandle:
#         outfilehandle.write("t,roll,pitch,yaw\n")
#         # s_ is for summed, i.e. s_roll = summed roll
#         last_dir = ""
#         for current_path in bag_paths:
#             this_dir = os.path.dirname(current_path)
#             if this_dir != last_dir:
#                 s_roll = 0.0
#                 s_pitch = 0.0
#                 s_yaw = 0.0
#                 last_dir = this_dir

#             with rosbag.Bag(current_path, 'r') as open_bag:
#                 for topic, msg, t in open_bag.read_messages("/imu/data"):
#                     t = t.to_sec()
#                     """
#                     s_roll = (s_roll + rad2deg(msg.angular_velocity.x)) % 360
#                     s_pitch = (s_pitch + rad2deg(msg.angular_velocity.y)) % 360
#                     s_yaw = (s_yaw + rad2deg(msg.angular_velocity.z)) % 360
#                     """
#                     s_roll = (s_roll + math.degrees(msg.angular_velocity.x)) % 360
#                     s_pitch = (s_pitch + math.degrees(msg.angular_velocity.y)) % 360
#                     s_yaw = (s_yaw + math.degrees(msg.angular_velocity.z)) % 360


#                     outfilehandle.write("{},{},{},{}\n".format(
#                         t,
#                         s_roll,
#                         s_pitch,
#                         s_yaw
#                     ))
#                 if verbose:
#                     print("Wrote:\nt: {}\nroll: {}\npitch: {}\nyaw: {}".format(
#                         t, s_roll, s_pitch, s_yaw
#                     ))


#     return [outpath] # Formatted as a list to play nice with downstream stuff


def concatinate_csv(csv_files, fields):
    ## Takes all csv files and concatinates their contents into a single list
    csv_data = []
    for csv_path in csv_files:
        with open(csv_path, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                payload = {f:row[f] for f in fields}
                csv_data.append(payload)
    return csv_data

def solve_opk(rpy_paths, gps_paths, outfile):
    ## Generates opk angles from input rpy readings and gps coordinates
    # Read in rpy and gps data
    rpy_data = concatinate_csv(rpy_paths, DEFAULT_RPY_FIELDS)
    gps_data = concatinate_csv(gps_paths, DEFAULT_GPS_FIELDS)
    print("{}{}{} roll, pitch, yaw angles will be considered".format(
        clr.SUCCESS, len(rpy_data), clr.RESET
    ))
    print("{}{}{} gps coordinates will be considered".format(
        clr.SUCCESS, len(gps_data), clr.RESET
    ))
    # Do conversion, function also handles writing to csv file
    opk_solution = rpy2opk_write(rpy_data, gps_data, outfile)
    print("{}{}{} matches were found".format(
        clr.SUCCESS, opk_solution, clr.RESET
    ))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="Rosbag to OPK angle calulator",
        description="Reads rosbag data and turns it into a csv containing omega, phi, and kappa angles"
    )

    parser.add_argument('-i',
                        '--input',
                        help="Path to input, can be directory or file. Inputting a directory will iterate over all files within the directory",
                        metavar="",
                        action="store"
    )

    parser.add_argument('-f',
                        '--fullrun',
                        help="Run LeGO-LOAM and then convert to opk angles",
                        action="store_true"
    )

    parser.add_argument('-L',
                        "--legoloam",
                        help="Just run LeGO LOAM",
                        action="store_true"
    )

    parser.add_argument('-m',
                        "--imu",
                        help="Get rpy angles from raw imu readings",
                        action="store_true")

    parser.add_argument('-a',
                        "--angles",
                        help="Just compute omega, phi, kappa angles from input csvs",
                        action="store_true")

    parser.add_argument('-g',
                        '--gps',
                        help="Path to gps file/directory",
                        metavar="",
                        action="store",
                        required=False
    )

    parser.add_argument('-o',
                        '--outfile',
                        help="Output file name, must be .csv",
                        metavar="",
                        action="store",
                        required=False
    )

    parser.add_argument("-v",
                        "--verbose",
                        help="Enable extra output messages",
                        action="store_true")

    args = parser.parse_args()

    # Handle output first before it becomes a problem later, after everything's run already
    # Better to have an error now lol
    if args.fullrun or args.angles:
        outfile = handle_output(args.outfile, DEFAULT_OPK_BASE_DIR)

    # Run LeGO-LOAM
    if args.legoloam:
        bag_files, bag_dirs = handle_input(args.input, ".bag")
        rpy_files = rpy_solution(bag_files, bag_dirs)

    if args.imu:
        bag_files, bag_dirs = handle_input(args.input, ".bag")
        rpy_files = imu2csv(bag_files, args.verbose)
        csvpath = rpy_files
    # Run Angle computation
    if args.angles:
        rpy_files, rpy_dirs = handle_input(args.input, ".csv")
    if args.fullrun or args.angles:
        gps_files, gps_dirs = handle_input(args.gps, ".csv", "No gps file provided")
        solve_opk(rpy_files, gps_files, outfile)



import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

# Path to the CSV file
# path = r".\raw-imu-solution\2024_06_06_15_58_21_546428.csv"
# path = r".\raw-imu-solution\2024_06_06_15_59_36_754748.csv"
path = csvpath[0]


# Load the data
data = pd.read_csv(path)

# Plot yaw
plt.figure()
data['yaw'].plot()
plt.title('Yaw')
plt.xlabel('Sample Index')
plt.ylabel('Yaw (degrees)')
plt.grid(True)
plt.show()

# Plot pitch
plt.figure()
data['pitch'].plot()
plt.title('Pitch')
plt.xlabel('Sample Index')
plt.ylabel('Pitch (degrees)')
plt.grid(True)
plt.show()

# Plot roll
plt.figure()
data['roll'].plot()
plt.title('Roll')
plt.xlabel('Sample Index')
plt.ylabel('Roll (degrees)')
plt.grid(True)
plt.show()
