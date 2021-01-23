#!/usr/bin/env python
# -*- coding: utf-8 -*-


from matplotlib import pyplot as plt
import math
import numpy as np


numOfWaypoints=100
xpoints = [0.25 ,9 ,9.75 ,9.75 ]
ypoints = [0.25 ,0.25 ,1 ,9 ,]

WaypointsX = np.zeros(((numOfWaypoints)*(len(xpoints)-1)))
WaypointsY = np.zeros(((numOfWaypoints)*(len(xpoints)-1)))
for i in range(len(xpoints)-3):
    x = np.linspace(xpoints[i],xpoints[i+1],numOfWaypoints)
    try:
        slope = ((ypoints[i+1])-(ypoints[i]))/((xpoints[i+1])-(xpoints[i]))
        coefficient = (ypoints[i])-(slope)*(xpoints[i])
        y = (slope)*x+coefficient
    except ZeroDivisionError:
        x = np.linspace(xpoints[i],xpoints[i],numOfWaypoints)
        y = np.linspace(ypoints[i],ypoints[i+1],numOfWaypoints)
    WaypointsX[((numOfWaypoints)*(i)):((numOfWaypoints)*(i+1))]=x
    WaypointsY[((numOfWaypoints)*(i)):((numOfWaypoints)*(i+1))]=y

# theta goes from 0 to 2pi
theta = np.linspace((1.5)*np.pi, (2)*np.pi, 100)

# the radius of the circle
r = np.sqrt((0.75)*(0.75))

# compute x1 and x2
x1 = r*np.cos(theta) + 9
x2 = r*np.sin(theta) + 1
#plt.plot(x1,x2)
WaypointsX[100:200] = x1
WaypointsY[100:200] = x2

i=2
x = np.linspace(xpoints[i],xpoints[i+1],numOfWaypoints)
try:
    slope = ((ypoints[i+1])-(ypoints[i]))/((xpoints[i+1])-(xpoints[i]))
    coefficient = (ypoints[i])-(slope)*(xpoints[i])
    y = (slope)*x+coefficient
except ZeroDivisionError:
    x = np.linspace(xpoints[i],xpoints[i],numOfWaypoints)
    y = np.linspace(ypoints[i],ypoints[i+1],numOfWaypoints)
WaypointsX[((numOfWaypoints)*(i)):((numOfWaypoints)*(i+1))]=x
WaypointsY[((numOfWaypoints)*(i)):((numOfWaypoints)*(i+1))]=y

plt.plot(WaypointsX[0:300],WaypointsY[0:300])
plt.show()