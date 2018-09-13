# -*- coding: utf-8 -*-
"""
Created on Tue Jan 12 15:08:15 2016

@author: admin-u5072689
"""

import numpy as np

def Absolute2RelativeXY(robot_abs,landmark_abs):
    
    '''
    Express Landmark's Coordinate on Robot Frame
    Input: robot's absolute coordinate [x,y,theta]
           landmarks absolute coordinate [x,y]
    Output: landmarks's relative coordinate with repect to robot frame [x,y] 
    i.e. landmark's measurement from robot
    '''
    
    x1 = robot_abs[0][0]
    y1 = robot_abs[1][0]
    theta1 = robot_abs[2][0]
    x2 = landmark_abs[0]    
    y2 = landmark_abs[1]
    

    #Calculate the difference with respect to world frame
    diff = [[x2-x1],
            [y2-y1],
            [1]]
    
    #R is the transition matrix to robot frame
    R = [[np.cos(-theta1), -np.sin(-theta1), 0],
         [np.sin(-theta1), np.cos(-theta1), 0],
         [0, 0, 1]]
         
    #Calculate Jacobian H1 with respect to X1
    H1 = [[-np.cos(theta1), -np.sin(theta1), -(x2-x1)*np.sin(theta1)+(y2-y1)*np.cos(theta1)],
          [np.sin(theta1), -np.cos(theta1), -(x2-x1)*np.cos(theta1)-(y2-y1)*np.sin(theta1)]]
         
    #Calculate Jacobian H2 with respect to X2
    H2 = [[np.cos(theta1), np.sin(theta1)],
          [-np.sin(theta1), np.cos(theta1)]]
         
    landmark_rel_xy = np.dot(R,diff)
    
    return [[landmark_rel_xy[0][0]],[landmark_rel_xy[1][0]]], H1, H2


if __name__ == "__main__":
    
    '''
    For self testing only
    '''    
    
    robot_abs = [2,1,1/4*(np.pi)]
    landmark_abs = [2,3]
    landmark_rel_xy, H1, H2 = Absolute2RelativeXY(robot_abs,landmark_abs)
    print (landmark_rel_xy)
    
