# -*- coding: utf-8 -*-
"""
Created on Thu Aug 11 10:53:05 2016

@author: admin-u5941570
"""

import numpy as np

def Relative2AbsoluteXY(robot_abs,landmark_meas_xy):

    '''
    Calcute Landmark's absolute coordinate
    Input: robot's absolute coordinate [x,y,theta]
           landmark's measurment with repect to robot frame [x,y]
    Output: landmarks's absolute coordinate  [x,y]
    '''
    
    x1 = robot_abs[0][0]
    y1 = robot_abs[1][0]
    theta1 = robot_abs[2][0]
    x2 = landmark_meas_xy[0]    
    y2 = landmark_meas_xy[1]
    
    landmark_meas = [[x2],
                     [y2],
                     [1]]
    
    #R is the transition matrix to robot frame
    R = [[np.cos(theta1), -np.sin(theta1), 0],
         [np.sin(theta1), np.cos(theta1), 0],
         [0, 0, 1]]
         
    #Calculate Jacobian H1 with respect to X1
    H1 = [[1, 0, -x2*np.sin(theta1)-y2*np.cos(theta1)],
          [0, 1,  x2*np.cos(theta1)-y2*np.sin(theta1)]]
         
    #Calculate Jacobian H2 with respect to X2
    H2 = [[np.cos(theta1), -np.sin(theta1)],
          [np.sin(theta1),  np.cos(theta1)]]
    #print R
    #print landmark_meas
    #print robot_abs        
    landmark_abs = np.array(np.dot(R,landmark_meas)) + np.array(robot_abs) 
    
    return [landmark_abs[0][0],landmark_abs[1][0]], H1, H2


if __name__ == "__main__":
    
    '''
    For self testing only
    '''    
    
    robot_abs = [[2],[1],[(1.0/4.0)*(np.pi)]]
    landmark_meas_xy = [1.4142, 1.4142]
    landmark_abs, H1, H2 = Relative2AbsoluteXY(robot_abs,landmark_meas_xy)
    print (landmark_abs)
    
