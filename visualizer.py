#!/usr/bin/env python3

import argparse
import math
import os
import sys

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import nvector as nv
from mpl_toolkits.mplot3d import Axes3D

import DataflashLog

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
alt = logdata.channels["GPS"]["RelAlt"].dictData

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

### animate ###

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

def animate(i):
    ax.clear()
    ax.plot_wireframe(x,y,z)
    ax.scatter(x[i], y[i], z[i], c='r', marker = 'o') # render copter
    ax.scatter(x[0], y[0], z[0], c='r', marker = 's') # render home

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('alt')
ani = animation.FuncAnimation(fig, animate, interval=100)
plt.show()
