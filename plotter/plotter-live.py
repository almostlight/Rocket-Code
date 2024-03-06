import matplotlib.pyplot as plt
import numpy as np
import csv 
from dataclasses import dataclass
import matplotlib.animation as anim 
from enum import Enum 
    

data = [[], [], [], [], [], [], [], [], [], [], [], [], [], [], [], [], []]

class ids(Enum):
    timestamp = int(2)
    ang_x = int(11)
    ang_z = int(13)
    corr_x = int(14)
    corr_z = int(15)


def pid_plot(axis, timestamps, angles, corrections, axis_id):
    axis.clear()
    axis.plot(timestamps, angles, linewidth=2, label='Angle', color='blue')
    axis.plot(timestamps, corrections, linewidth=2, label='PID output', alpha=0.5, color='orange')
    xmin = float(min(timestamps))
    xmax = float(max(timestamps))
    ymin = float(min(corrections))
    ymax = float(max(corrections))
    axis.set_xticks(np.arange(xmin, xmax+0.1, step=(xmax-xmin)))
    #axis.set_yticks(np.arange(-round(max(ymin, ymax)), round(max(ymin, ymax))+0.1, step=1)) #step=(ymax-ymin)/10
    axis.set_yticks(np.arange(-6.0, 6.1, step=1)) 
    axis.set_xlabel("time (s)")
    axis.set_ylabel("angle (degrees)")
    #axis.axhline(y=0, alpha=0.5)
    axis.grid(True, which='both')
    axis.set_title(axis_id + "-axis")
    axis.legend()

def update_plot(axis, timestamps, angles, corrections):
    axis.plot(timestamps, angles, linewidth=2, label='Angle', color='blue')
    axis.plot(timestamps, corrections, linewidth=2, label='PID output', alpha=0.5, color='orange')
    ymin = float(min(corrections))
    ymax = float(max(corrections))
    axis.set_yticks(np.arange(-round(max(ymin, ymax)), round(max(ymin, ymax))+0.1, step=1))

fig, plots = plt.subplots(2)
plt.ion()

with open("data.csv", 'r') as datafile:
    reader = csv.reader(datafile, delimiter=',')
    firstrow = True 
    first_timestamp = 0
    for row in reader: 
        if firstrow == True:
            firstrow = False 
        else: 
            if len(data[ids.timestamp.value])==0:
                first_timestamp = float(float(row[ids.timestamp.value]))
                data[ids.timestamp.value].append(0.0)
            else: 
                data[ids.timestamp.value].append((float(float(row[2]))-first_timestamp)/1000)
            data[ids.ang_x.value].append(float(row[ids.ang_x.value]))
            data[ids.ang_z.value].append(float(row[ids.ang_z.value]))
            data[ids.corr_x.value].append(float(row[ids.corr_x.value]))
            data[ids.corr_z.value].append(float(row[ids.corr_z.value]))

            if len(data[ids.timestamp.value])>20: 
                for list in data:
                    if len(data[data.index(list)])>0:     #   no empty lists 
                        data[data.index(list)].pop(0)
            
            fig.tight_layout() 
            if len(data[ids.timestamp.value])>1:
                print(data[ids.timestamp.value][-1]) 
                pid_plot(plots[0], data[ids.timestamp.value], data[ids.ang_x.value], data[ids.corr_x.value], "x")
                pid_plot(plots[1], data[ids.timestamp.value], data[ids.ang_z.value], data[ids.corr_z.value], "y")
                plt.draw()
                plt.pause(0.001)       
datafile.close()

plt.ioff()
fig.savefig("figure") 
plt.show()
