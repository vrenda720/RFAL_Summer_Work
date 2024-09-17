'''
Misc. timestamp tools for working with all the wacky time stuff

'''

from datetime import datetime
import os, sys, json


DT_FORMAT = "%Y-%m-%d %H:%M:%S.%f"

def est2epoch(est_string):
    dt = datetime.strptime(est_string, DT_FORMAT)
    epoch_time = (dt - datetime(1970,1,1)).total_seconds()
    return epoch_time + 18000   # 18000 is the est offset

def utc2epoch(utc_string):
    dt = datetime.strptime(utc_string, DT_FORMAT)
    epoch_time = (dt - datetime(1970,1,1)).total_seconds()
    return epoch_time

def epoch2utc(timestamp):
    dt = datetime.fromtimestamp(timestamp)
    return dt.strftime(DT_FORMAT)
