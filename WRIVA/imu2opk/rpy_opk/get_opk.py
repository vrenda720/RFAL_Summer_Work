from rpy_opk import projections, time_tool as tt

GPS_FIELDS = ["Measurement_DateTime", "GPS_lat", "GPS_lon", "GPS_alt"]
RPY_FIELDS = ["t", "roll", "pitch", "yaw"]
# IMU tends to update many times per second, so time tolerance can be really small
TIME_TOLERANCE = 0.1    

def rpy2opk_write(rpy_array, gps_array, filename):
    total_matches = 0
    with open(filename, 'w') as f:
        f.write("timestamp,latitude,longitude,altitude,roll,pitch,yaw,omega,phi,kappa\n")
        for gps_instance in gps_array:
            gps_time = tt.utc2epoch(gps_instance["Measurement_DateTime"])
            for rpy_instance in rpy_array:
                rpy_time = float(rpy_instance["t"])
                if rpy_time > (gps_time - TIME_TOLERANCE) and rpy_time < (gps_time + TIME_TOLERANCE):
                    this_lat   = float(gps_instance["GPS_lat"])
                    this_lon   = float(gps_instance["GPS_lon"])
                    this_alt   = float(gps_instance["GPS_alt"])
                    this_roll  = float(rpy_instance["roll"])
                    this_pitch = float(rpy_instance["pitch"])
                    this_yaw   = float(rpy_instance["yaw"])
                
                    this_omega, this_phi, this_kappa = projections.ypr_to_opk(
                        this_lat,
                        this_lon,
                        this_alt,
                        this_yaw,
                        this_pitch,
                        this_roll
                    )

                    payload = "{},{},{},{},{},{},{},{},{},{}\n".format(
                        gps_time,
                        this_lat,
                        this_lon,
                        this_alt,
                        this_roll,
                        this_pitch,
                        this_yaw,
                        this_omega,
                        this_phi,
                        this_kappa
                    )                    
                    f.write(payload)
                    total_matches += 1
                    break    # Continue to next gps instance so there aren't like 10,000 repeats
    return total_matches