#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tutorial_tmcn.msg import GlobWaypoints,GoalIndex,TransformedGoalStates
from tf.transformations import euler_from_quaternion

## args:
        # goal_index : Goal index on the REF PATH, is needed to create a local path
        # RefWaypointsX: Ref Path 
        # RefWaypointsY: Ref Path
        # CurrX [m]: Current x posiiton of the vehicle
        # CurrY [m]: Current y poisiton of the vehicle
        # Theta [rad]: Yaw angle
## outputs:        
        # Transformed_goal_state: [x_points y_points heading]   num_pathsx3

class TransGoalStateClass(object):

    def __init__(self):
        rospy.Subscriber('GoalIndexTopic',GoalIndex,self.get_goal_index,queue_size=10)
        rospy.Subscriber('GlobalPath',GlobWaypoints,self.get_glob_path,queue_size=10)
        rospy.Subscriber('odom',Odometry,self.get_states,queue_size=10)
        self.pub =rospy.Publisher('TransGoalStatesTopic',TransformedGoalStates,queue_size=1)
        
        self.globwaypoints = GlobWaypoints()
        self.x = 0
        self.y = 0
        self.theta = 0
        self.num_paths = 7
        self.goal_state_set = np.zeros((self.num_paths,3))
        self.goal_state = TransformedGoalStates()
        self.path_offset = 0.4
        self.heading = 0
        self.rate = rospy.Rate(4)
        self.goalIndex = 0

    def get_goal_index(self,msg):
        
        self.goalIndex = msg.goalindex

    def get_glob_path(self,msg):
        self.globwaypoints.globwaypointsx = msg.globwaypointsx
        self.globwaypoints.globwaypointsy = msg.globwaypointsy

    def get_states(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        self.theta = yaw 
        

    def loop(self):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('GlobalPath',GlobWaypoints)
            except:
                pass
        while not rospy.is_shutdown():
            self.main()
            self.pub.publish(self.goal_state)
            self.rate.sleep()

        
    def calculate_heading_angle(self):
        ## CALCULATES HEADING ANGLE OF THE GOAL POINT
        if self.goalIndex == len(self.globwaypoints.globwaypointsx):
            self.goalIndex = self.goalIndex-1
            delta_x = (self.globwaypoints.globwaypointsx[self.goalIndex+1])-(self.globwaypoints.globwaypointsx[self.goalIndex])
            delta_y = (self.globwaypoints.globwaypointsy[self.goalIndex+1])-(self.globwaypoints.globwaypointsy[self.goalIndex])
        else:
            delta_x = (self.globwaypoints.globwaypointsx[self.goalIndex+1])-(self.globwaypoints.globwaypointsx[self.goalIndex])
            delta_y = (self.globwaypoints.globwaypointsy[self.goalIndex+1])-(self.globwaypoints.globwaypointsy[self.goalIndex])
        
        #print("delta_y:",delta_y)
        try:
            angle = math.atan((delta_y)/(delta_x))
        except ZeroDivisionError:
            angle = (math.pi)/(2) 
        return angle

    def calc_trans_rot(self):
        """TRANSLATION AND ROTATION TO HAVE GOAL_STATE_SET W.R.T VEHICLE FRAME"""
        
        goal_state_localX = self.globwaypoints.globwaypointsx[self.goalIndex]-self.x
        goal_state_localY = self.globwaypoints.globwaypointsy[self.goalIndex]-self.y
        theta = -self.theta
        goal_x = (math.cos(theta))*(goal_state_localX)-(math.sin(theta))*(goal_state_localY)
        goal_y = (math.cos(theta))*(goal_state_localY)+(math.sin(theta))*(goal_state_localX)
        self.heading = self.calculate_heading_angle()
        goal_t = self.heading-self.theta

        if goal_t > math.pi :
            goal_t = goal_t - (2)*(math.pi)
        if goal_t < -math.pi :
            goal_t = goal_t + (2)*(math.pi)

        for i in range(self.num_paths):
            offset = (i-round(self.num_paths/2))*(self.path_offset)
            x_offset = (offset)*(math.cos(goal_t+math.pi/2))
            y_offset = (offset)*(math.sin(goal_t+math.pi/2))
            
            self.goal_state_set[i,:] = [goal_x+x_offset, goal_y+y_offset,goal_t]
        self.goal_state.x = self.x
        self.goal_state.y = self.y
        self.goal_state.goal_state_vehicle_framex = self.goal_state_set[:,0]
        self.goal_state.goal_state_vehicle_framey = self.goal_state_set[:,1]
        self.goal_state.goal_state_vehicle_frametheta = self.goal_state_set[:,2]

    def main(self):
        self.calc_trans_rot()
        self.trans_into_glob_frame()

    def trans_into_glob_frame(self):
        """TRANSLATION AND ROTATION TO HAVE GOAL_STATE_SET W.R.T GLOBAL FRAME
            TRANSFORMS FROM VEHICLE FRAME INTO GLOBAL FRAME
        """
        ## TRANSFORMS INTO GLOBAL FRAME
        x1 = np.array(self.goal_state_set[:,0])
        y1 = np.array(self.goal_state_set[:,1])
        goal_state_set_heading = self.goal_state_set[:,2]
        x2 = (math.cos(self.theta))*(x1)-(math.sin(self.theta))*(y1)
        y2 = (math.cos(self.theta))*(y1)+(math.sin(self.theta))*(x1)
        x2 =self.x + x2
        y2 = self.y + y2
        self.goal_state.theta = self.theta
        self.heading = goal_state_set_heading+self.theta
        transformed_goal_state = np.dstack((x2,y2,self.heading),)
        transformed_goal_state = transformed_goal_state[0,:,:]
        self.goal_state.goal_state_global_framex = transformed_goal_state[:,0]
        self.goal_state.goal_state_global_framey = transformed_goal_state[:,1]
        self.goal_state.goal_state_global_frametheta = transformed_goal_state[:,2]


if __name__ == "__main__":
    try:
        rospy.init_node('TransformedGoalStatev3',anonymous=True)
        transGoalStateObj = TransGoalStateClass()
        transGoalStateObj.loop()
    except rospy.ROSInterruptException():
        pass
