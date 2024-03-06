import matplotlib.pyplot as plt
import numpy as np
import csv 

Timestamp = []
Pressure = []
Altitude = []
AngX = [] 
AngZ = [] 
PID_X = [] 
PID_Z = []

with open("data.csv", 'r') as datafile:
    reader = csv.reader(datafile, delimiter=',')
    firstrow = True 
    first_timestamp = 0
    for row in reader: 
        if firstrow == True:
            firstrow = False 
        elif float(row[1]) == 1: 
            if len(Timestamp)==0:
                first_timestamp = float(row[2])
                Timestamp.append(0.0)
            else: 
                Timestamp.append((float(row[2])-first_timestamp)/1000)
            Pressure.append(float(row[3]))
            Altitude.append(float(row[4]))
            AngX.append(float(row[11]))
            AngZ.append(float(row[13]))
            PID_X.append(float(row[14]))
            PID_Z.append(float(row[15]))
datafile.close()

def pid_plot(axis, x_arr, angles, corrections, axis_id):
    axis.plot(x_arr, angles, linewidth=2, label='Angle')
    axis.plot(x_arr, corrections, linewidth=2, label='PID output', alpha=0.5)

    xmin = float(min(x_arr))
    xmax = float(max(x_arr))
    ymin = float(min(corrections))
    ymax = float(max(corrections))
    axis.set_xticks(np.arange(xmin, xmax, step=1.0))
    axis.set_yticks(np.arange(-round(max(ymin, ymax)), round(max(ymin, ymax))+0.1, step=1)) #step=(ymax-ymin)/10
    axis.set_xlabel("time (s)")
    axis.set_ylabel("angle (degrees)")
    #axis.axhline(y=0, alpha=0.5)
    axis.grid(True, which='both')
    axis.set_title(axis_id + "-axis")
    axis.legend()

fig, axs = plt.subplots(2)

pid_plot(axs[0], Timestamp, AngX, PID_X, "x")
pid_plot(axs[1], Timestamp, AngZ, PID_Z, "y")

fig.tight_layout()
fig.savefig("figure")
plt.show()
