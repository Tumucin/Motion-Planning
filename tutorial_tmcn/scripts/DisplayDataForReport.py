#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import math
import numpy as np
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tutorial_tmcn.msg import GlobWaypoints,GoalIndex,TransformedGoalStates,LocalWaypoints,AngularVelocity
from tf.transformations import euler_from_quaternion

class DisplayData(object):

    def __init__(self):
        rospy.Subscriber('odom',Odometry,self.get_states,queue_size=1)
        self.xpoints = [0.25 ,9 ,9.75 ,9.75 ,9 ,1 ,0.25 ,0.25]
        self.ypoints = [0.25 ,0.25 ,1 ,9 ,9.75 ,9.75 ,9 ,0.25]
        #self.xpoints = [0.25 ,9 ,9.75 ,9.75 ,9 ,6 ,5.75 ,5.75]
        #self.ypoints = [0.25 ,0.25 ,1 ,3 ,3.75 ,3.75 ,4.25 ,9]
        self.x = 0
        self.y = 0
        self.theta = 0
        self.rate = rospy.Rate(3)

        rospy.Subscriber('LocalWaypoints',LocalWaypoints, self.get_ref_path,queue_size=1)
        rospy.Subscriber('angular_velocity',AngularVelocity,self.get_angular_vel,queue_size=1)
        self.waypoints = LocalWaypoints()
        rospy.Subscriber('GlobalPath',GlobWaypoints,self.get_glob_path,queue_size=1)
        self.globwaypoints = GlobWaypoints()
        self.angularVelocity = []
        self.LineHeadingError = []
        self.headingErr = []
        self.time = []
        self.xhistory = []
        self.yhistory = []
        
        rospy.Subscriber('TransGoalStatesTopic',TransformedGoalStates,self.get_trans_goal_states,queue_size=1)
        self.transformed_goal_state = TransformedGoalStates()

        self.derivtheta = 0
        self.prevglobaltheta = 0

    def get_angular_vel(self,msg):
        x = msg.angularVelocity
        delta = math.atan(((x)*(0.106))/(0.5))
        self.angularVelocity.append(delta)
        self.LineHeadingError.append(msg.LineHeadingError)
        self.headingErr.append(msg.headingErr)
        time =rospy.get_time()
        self.time.append(time)

    def get_glob_path(self,msg):
        self.globwaypoints.globwaypointsx = msg.globwaypointsx
        self.globwaypoints.globwaypointsy = msg.globwaypointsy

    def loop(self):
        while not rospy.is_shutdown():
            self.drawLine()
            self.rate.sleep()

    def get_states(self,msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.xhistory.append(self.x)
        self.yhistory.append(self.y)

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        self.theta = yaw

    def get_ref_path(self,msg):
        self.waypoints.localwaypointsx = msg.localwaypointsx
        self.waypoints.localwaypointsy = msg.localwaypointsy
        #self.calculate_coeff()
        #plt.plot(self.waypoints.localwaypointsx,self.waypoints.localwaypointsy)
        #plt.ylim(-3,3)
        #plt.xlim(0,10)
        #plt.show()

    def drawLine(self):
        try:
            theta = np.linspace(0, 2*np.pi, 100)
            r = 0.35
            x1 = r*np.cos(theta) + 4
            x2 = r*np.sin(theta) +0.25
            plt.figure(1)
            plt.plot()
            plt.plot(self.globwaypoints.globwaypointsx,self.globwaypoints.globwaypointsy,'k',label='GlobalPath',linewidth=4)
            plt.plot(self.waypoints.localwaypointsx,self.waypoints.localwaypointsy,'y',label='LocalPath', linewidth=4)
            plt.plot(self.x,self.y,'bo',label= 'CurrentPosition')
            plt.plot(x1,x2,label= 'Obstacle')
            
            #for i in range(7):
            plt.plot(self.transformed_goal_state.goal_state_global_framex,self.transformed_goal_state.goal_state_global_framey,'go',label='GoalStates')
            
            #plt.plot(self.x,self.derivtheta,'o-',label='DerivativeTheta')
            plt.plot(self.xpoints, self.ypoints, 'ro', label = 'Waypoints')
            plt.plot(self.xhistory, self.yhistory,'m', label = "ActualPath", linewidth=2.5)
            plt.xlim(0,10)
            plt.ylim(-3,12)
            plt.legend()
            plt.grid()
            plt.show()
            """plt.figure(2)
            plt.plot(self.time,self.angularVelocity, label= 'TotalSteering')
            plt.legend()
            plt.grid()
            plt.figure(3)
            plt.plot(self.time, self.LineHeadingError,label= 'LineHeading')
            plt.legend()
            plt.grid()
            plt.figure(4)
            plt.plot(self.time, self.headingErr, label= 'HeadingError')
            plt.legend()
            plt.grid()
            #plt.show()"""
            self.prevglobaltheta = self.transformed_goal_state.goal_state_global_frametheta[0]
        except IndexError:
            pass

    def get_trans_goal_states(self,msg):
        self.transformed_goal_state.goal_state_vehicle_framex = msg.goal_state_vehicle_framex
        self.transformed_goal_state.goal_state_vehicle_framey = msg.goal_state_vehicle_framey
        self.transformed_goal_state.goal_state_vehicle_frametheta = msg.goal_state_vehicle_frametheta
        self.transformed_goal_state.goal_state_global_framex = msg.goal_state_global_framex
        self.transformed_goal_state.goal_state_global_framey = msg.goal_state_global_framey
        self.transformed_goal_state.goal_state_global_frametheta = msg.goal_state_global_frametheta
        self.derivtheta = self.transformed_goal_state.goal_state_global_frametheta[0]-self.prevglobaltheta


    
if __name__ == "__main__":
    try:
        rospy.init_node('DisplayData',anonymous=True)
        displaydata = DisplayData()
        displaydata.loop()
    except rospy.ROSInterruptException:
        pass