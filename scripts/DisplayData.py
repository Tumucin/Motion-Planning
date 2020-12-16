#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import math
import numpy as np
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tutorial_tmcn.msg import GlobWaypoints,GoalIndex,TransformedGoalStates,LocalWaypoints
from tf.transformations import euler_from_quaternion

class DisplayData(object):

    def __init__(self):
        rospy.Subscriber('odom',Odometry,self.get_states,queue_size=1)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.rate = rospy.Rate(2)

        rospy.Subscriber('LocalWaypoints',LocalWaypoints, self.get_ref_path,queue_size=1)
        self.waypoints = LocalWaypoints()
        rospy.Subscriber('GlobalPath',GlobWaypoints,self.get_glob_path,queue_size=10)
        self.globwaypoints = GlobWaypoints()

        rospy.Subscriber('TransGoalStatesTopic',TransformedGoalStates,self.get_trans_goal_states,queue_size=1)
        self.transformed_goal_state = TransformedGoalStates()

        self.derivtheta = 0
        self.prevglobaltheta = 0

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

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

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
            plt.plot(self.x,self.y,'o-',label= 'CurrentPosition')
            plt.plot(self.waypoints.localwaypointsx,self.waypoints.localwaypointsy,label='LocalPath')
            for i in range(7):
                plt.plot(self.transformed_goal_state.goal_state_global_framex[i],self.transformed_goal_state.goal_state_global_framey[i],'o-')
            
            plt.plot(self.x,self.derivtheta,'o-',label='DerivativeTheta')
            plt.plot(self.globwaypoints.globwaypointsx,self.globwaypoints.globwaypointsy,label='GlobalPath')
            plt.legend()
            plt.grid()
            plt.xlim(0,10)
            plt.ylim(0,10)
            plt.show()
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