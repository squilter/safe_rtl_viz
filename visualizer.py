#!/usr/bin/env python3

import argparse
import sys

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import nvector as nv
from mpl_toolkits.mplot3d import Axes3D

import DataflashLog
from path_cleanup import Path

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

### animate ###

return_path = Path( [ (x[0],y[0],z[0]) ] )
path_len = []
counter = 10 # only render memory usage every 10 frames

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
mem_ax = fig.add_subplot(5,2,10)

def animate(i):
    try:
        return_path.append_if_far_enough( (x[i], y[i], z[i]) )
        return_path.routine_cleanup()
    except IndexError:
        pass # pause everything when it ends. Let the user choose when to end the program
    ax.clear()

    ## plot flown whole path
    # ax.plot_wireframe(x,y,z)
    ## plot return path currently in memory
    ax.plot_wireframe([k[0] for k in return_path.path], [k[1] for k in return_path.path], [k[2] for k in return_path.path], color='green')
    ## plot hypothetical return path if RTL activated now
    flyback_path = return_path.get_flyback_path()
    ax.plot_wireframe([k[0] for k in flyback_path], [k[1] for k in flyback_path], [k[2] for k in flyback_path], color='red')
    ## render copter
    try:
        ax.scatter(x[i], y[i], z[i], c='r', marker = 'o')
    except:
        pass
    ## render memory usage
    global counter
    counter = counter+1
    if counter > 10:
        counter = 0
        path_len.append(len(return_path.path))
        mem_ax.clear()
        mem_ax.plot(path_len)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('alt')
ani = animation.FuncAnimation(fig, animate, interval=10)
# ani.save('vid.mp4', metadata={'artist':'squilter'}, fps = 10)
try:
    plt.show()
except AttributeError:
    sys.exit(0)
