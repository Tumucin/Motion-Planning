#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from tutorial_tmcn.msg import GlobWaypoints
from matplotlib import pyplot as plt
import math
import numpy as np
from nav_msgs.msg import Odometry

class GlobalPath(object):
    
    def __init__(self):
        #self.xpoints = [0.25 ,9 ,9.553 ,9.75 ,9.75 ,9.54 ,9 ,1 ,0.4598 ,0.25 ,0.25]
        #self.ypoints = [0.25 ,0.25 ,0.4935 ,1 ,9 ,9.52 ,9.75 ,9.75 ,9.52 ,9 ,0.25]
        self.xpoints = [0.25 ,9 ,9.75 ,9.75 ,9 ,1 ,0.25 ,0.25]
        self.ypoints = [0.25 ,0.25 ,1 ,9 ,9.75 ,9.75 ,9 ,0.25]
        #self.xpoints = [0.25 ,9 ,9.75 ,9.75 ,9 ,6 ,5.75 ,5.75]
        #self.ypoints = [0.25 ,0.25 ,1 ,3 ,3.75 ,3.75 ,4.25 ,9]
        self.numOfWaypoints = 100

        self.pub = rospy.Publisher('GlobalPath',GlobWaypoints,queue_size=1) # Publishes to GlobalPath
        rospy.Subscriber('odom',Odometry,self.get_states,queue_size=1)
        self.x = 0
        self.y = 0
        self.rate = rospy.Rate(7)
        self.waypoints = GlobWaypoints()

    def get_states(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.rate.sleep()
    
    def main(self,xpoints,ypoints,numOfWaypoints):
        """ Creates global path using waypoints
        args:
                xpoints: X positions of the waypoints [m]
                ypoints: Y positions of the waypoints [m]
                numOfWaypoints: Number of the samples between two waypoints  
        returns:
                WaypointsX: Global path (x points) [m]
                WaypointsY: Global path (y points) [m]
        """
        WaypointsX = np.zeros(((numOfWaypoints)*(len(xpoints)-1)))
        WaypointsY = np.zeros(((numOfWaypoints)*(len(xpoints)-1)))

        for i in range(len(xpoints)-1):
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

        return WaypointsX, WaypointsY

    def loop(self):
        self.waypoints.globwaypointsx,self.waypoints.globwaypointsy = self.main(self.xpoints,self.ypoints,self.numOfWaypoints)
        while not rospy.is_shutdown():
            self.pub.publish(self.waypoints)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node('CreateGlobalPathv4',anonymous=True)
        globalPathobj = GlobalPath()
        globalPathobj.loop()
    except rospy.ROSInterruptException():
        pass
