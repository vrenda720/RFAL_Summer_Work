import csv, os, sys, json
import projections
import time_tool as th
import pdb

GPS_FIELDS = ["Measurement_DateTime", "GPS_lat", "GPS_lon", "GPS_alt"]
RPY_FIELDS = ["t", "roll", "pitch", "yaw"]
TIME_TOLERANCE = 1.0


class get_opk:

    def __init__(self, gps_path, rpy_path):
        self.gps_path = gps_path
        self.rpy_path = rpy_path

        self.gps_data = self.read_csv(gps_path, GPS_FIELDS)
        self.rpy_data = self.read_csv(rpy_path, RPY_FIELDS)

    def read_csv(self, csv_path, fields):
        out_data = []
        with open(csv_path, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                payload = {f:row[f] for f in fields}
                out_data.append(payload)
        return out_data
    
    def map_time(self, gps_array, rpy_array):
        # Take the two timestamps from array_1 and array_2 and make a single 
        # Index array by timestamp, use the timestamps from both arrays to find readings within 1s of each other

        out_array = []

        for gps_item in gps_array:

            gps_time = th.utc2epoch(gps_item["Measurement_DateTime"])
            for rpy_item in rpy_array:
                rpy_time = float(rpy_item["t"])
                if rpy_time > (gps_time - TIME_TOLERANCE) and rpy_time < (gps_time + TIME_TOLERANCE):
                    this_lat = float(gps_item["GPS_lat"])
                    this_lon = float(gps_item["GPS_lon"])
                    this_alt = float(gps_item["GPS_alt"])
                    this_roll = float(rpy_item["roll"])
                    this_pitch = float(rpy_item["pitch"])
                    this_yaw = float(rpy_item["pitch"])
                    
                    this_omega, this_phi, this_kappa = projections.ypr_to_opk(
                        this_lat,
                        this_lon,
                        this_alt,
                        this_yaw,
                        this_pitch,
                        this_roll
                    )
                    
                    payload = {
                        "timestamp": gps_time,
                        "latitude": this_lat,
                        "longitude": this_lon,
                        "altitude": this_alt,
                        "roll": this_roll,
                        "pitch": this_pitch,
                        "yaw": this_yaw,
                        "omega": this_omega,
                        "phi": this_phi,
                        "kappa": this_kappa
                    }

                    out_array.append(payload)
    
        self.writeout(out_array)
    
    def writeout(self, out_array, csvfilename):
        with open(csvfilename, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=' ', 
                                   quotechar='|', quoting=csv.QUOTE_MINIMAL)
            for item in out_array:
                csvwriter.writerow(item)

if __name__ == "__main__":
    gps_path = "/media/wriva/ext_mnt/temp/GPSL0005.CSV"
    rpy_path = "/home/rfal/Desktop/lego-loam-good/11-17-2023-08-06/1.csv"

    session = get_opk(gps_path, rpy_path)
    session.map_time(session.gps_data, session.rpy_data)
