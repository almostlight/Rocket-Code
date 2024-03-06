import matplotlib.pyplot as plt
import numpy as np
import csv 
from dataclasses import dataclass
import matplotlib.animation as anim 
from enum import Enum 


total_time = 10         # seconds 
jump = 10   # how many datapoints per refresh


data = [[], [], [], [], [], [], [], [], [], [], [], [], [], [], [], [], []]

class ids(Enum):
    timestamp = int(2)
    ang_x = int(11)
    ang_z = int(13)
    corr_x = int(14)
    corr_z = int(15)


def pid_plot(axis, timestamps, angles, corrections, axis_id):
    #axis.plot(timestamps, angles, linewidth=2, label='Angle', color='blue')
    #axis.plot(timestamps, corrections, linewidth=2, label='PID output', alpha=0.5, color='orange')
    xmin = float(min(timestamps))
    xmax = float(max(timestamps))
    ymin = float(min(corrections))
    ymax = float(max(corrections))
    axis.set_xticks(np.arange(xmin, xmax+0.1, step=1)) 
    axis.set_yticks(np.arange(-6.0, 6.1, step=1)) 
    axis.set_xlabel("time (s)")
    axis.set_ylabel("angle (degrees)")
    axis.grid(True, which='both')
    axis.set_title(axis_id + "-axis")
    axis.legend()

with open("data.csv", 'r') as datafile:
    reader = csv.reader(datafile, delimiter=',')
    firstrow = True 
    first_timestamp = 0
    for row in reader: 
        if firstrow == True:
            firstrow = False 
        elif float(row[1]) == 1:  
            if len(data[ids.timestamp.value])==0:
                first_timestamp = float(float(row[ids.timestamp.value]))
                data[ids.timestamp.value].append(0.0)
            else: 
                data[ids.timestamp.value].append((float(float(row[2]))-first_timestamp)/1000)
            data[ids.ang_x.value].append(float(row[ids.ang_x.value]))
            data[ids.ang_z.value].append(float(row[ids.ang_z.value]))
            data[ids.corr_x.value].append(float(row[ids.corr_x.value]))
            data[ids.corr_z.value].append(float(row[ids.corr_z.value]))
datafile.close()

fig, plots = plt.subplots(2) 

ln0, = plots[0].plot(data[ids.timestamp.value], data[ids.ang_x.value], linewidth=2, label='Angle', color='blue')
ln1, = plots[0].plot(data[ids.timestamp.value], data[ids.corr_x.value], linewidth=2, label='PID output', alpha=0.5, color='orange')
ln2, = plots[1].plot(data[ids.timestamp.value], data[ids.ang_z.value], linewidth=2, label='Angle', color='blue')
ln3, = plots[1].plot(data[ids.timestamp.value], data[ids.corr_z.value], linewidth=2, label='PID output', alpha=0.5, color='orange')

timestamp, ang_data_x, corr_data_x, ang_data_z, corr_data_z = [], [], [], [], [] 
pid_plot(plots[0], data[ids.timestamp.value], data[ids.ang_x.value], data[ids.corr_x.value], 'x')
pid_plot(plots[1], data[ids.timestamp.value], data[ids.ang_z.value], data[ids.corr_z.value], 'z')
plt.tight_layout()

def animation_init():
    print()

def animation_frame(i):
    for n in range (i, i+(jump-1)):
        if len(data[ids.timestamp.value]) > n:
            timestamp.append(data[ids.timestamp.value][n])
            ang_data_x.append(data[ids.ang_x.value][n])
            corr_data_x.append(data[ids.corr_x.value][n])
            ang_data_z.append(data[ids.ang_z.value][n])
            corr_data_z.append(data[ids.corr_z.value][n])
            print(data[ids.timestamp.value][n], data[ids.corr_x.value][n], '\n')
    ln0.set_data(timestamp, ang_data_x)
    ln1.set_data(timestamp, corr_data_x)
    ln2.set_data(timestamp, ang_data_z)
    ln3.set_data(timestamp, corr_data_z)
    return ln0, ln1, ln2, ln3

animation = anim.FuncAnimation(fig, func=animation_frame, frames=np.arange(0,len(data[ids.timestamp.value]),jump), 
                               interval=(total_time*jump*1000/len(data[ids.timestamp.value])), repeat=False, init_func=animation_init())

#fig.savefig("figure") 
#animation.save("figure.gif", writer='imagemagick')
plt.show()
