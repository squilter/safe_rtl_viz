#!/usr/bin/env python3

import argparse
import math
import os
import sys
import time

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import nvector as nv
from mpl_toolkits.mplot3d import Axes3D

import DataflashLog
import path_cleanup

### tuning variables ###

position_delta = 2. # how many meters to move before appending a new position to return_path
rdp_epsilon = 0.5 # The tuning variable used in the simplification step
cleanup_length = 15 # The number of points stored in memory that triggers the cleanup method

### setup ###

parser = argparse.ArgumentParser(description='Analyze an APM Dataflash log for known issues')
parser.add_argument('logfile', type=argparse.FileType('r'), help='path to Dataflash log file (or - for stdin)')
parser.add_argument('-f', '--format',  metavar='', type=str, action='store', choices=['bin','log','auto'], default='auto', help='log file format: \'bin\',\'log\' or \'auto\'')
parser.add_argument('-s', '--skip_bad', metavar='', action='store_const', const=True, help='skip over corrupt dataflash lines')
args = parser.parse_args()
logdata = DataflashLog.DataflashLog(args.logfile.name, format=args.format, ignoreBadlines=args.skip_bad) # read log

if "GPS" not in logdata.channels:
    print("No GPS log data")
    sys.exit(0)

### Convert from lat/lon to meters ###

wgs84 = nv.FrameE(name='WGS84')

lat = logdata.channels["GPS"]["Lat"].dictData
lon = logdata.channels["GPS"]["Lng"].dictData
alt = []
if "RelAlt" in logdata.channels["GPS"]:
    alt = logdata.channels["GPS"]["RelAlt"].dictData
elif "Alt" in logdata.channels["GPS"]:
    alt = logdata.channels["GPS"]["Alt"].dictData
else:
    print("No GPS log data")
    sys.exit(0)

first_index = min(lat.keys())

home = wgs84.GeoPoint(latitude=lat[first_index], longitude=lon[first_index], z=alt[first_index], degrees=True)

x, y, z = [], [], []
for i in lat.keys():
    position = wgs84.GeoPoint(latitude=lat[i], longitude=lon[i], z=alt[i], degrees=True)
    ned = nv.diff_positions(home, position)
    frame_N = nv.FrameN(home)
    ned = ned.change_frame(frame_N)
    ned = ned.pvector.ravel()
    x.append(ned[0])
    y.append(ned[1])
    z.append(alt[i])

### algorithm part ###

return_path = [ (x[0],y[0],z[0]) ]

def update_return_path(p):
    global return_path
    x,y,z = p
    x_old, y_old, z_old = return_path[-1]
    # if more than 1 meter since old position, add it to return path
    if (x-x_old)**2+(y-y_old)**2+(z-z_old)**2 >= position_delta**2: # we square the constant side, rather than taking the sqrt every time. more efficient.
        return_path.append(p)
    else:
        # don't bother with cleanup steps if we haven't changed anything.
        return

    if len(return_path) >= cleanup_length: #TODO maybe it makes more sense to run the cleanup more often, but record which comparisons have already been made (so each cleanup is quicker)
        #TODO what happens if the cleanup fails and we are at risk of running out of memory? Maybe try again with more aggresive parameters, but we cannot waste too much time.
        pruning_occured = True
        while pruning_occured:
            return_path, pruning_occured = path_cleanup.cleanup(return_path,pos_delta=position_delta, rdp_eps=rdp_epsilon)

### animate ###

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

def animate(i):
    try:
        update_return_path((x[i], y[i], z[i]))
    except IndexError:
        time.sleep(3)
        sys.exit(0)
    ax.clear()
    # comment out lines below to choose what to render
    ax.plot_wireframe([k[0] for k in return_path], [k[1] for k in return_path], [k[2] for k in return_path], color='red') # plot calculated return path
    ax.plot_wireframe(x,y,z) # plot whole path
    ax.scatter(x[i], y[i], z[i], c='r', marker = 'o') # render copter

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('alt')
ani = animation.FuncAnimation(fig, animate, interval=100)
plt.show()
