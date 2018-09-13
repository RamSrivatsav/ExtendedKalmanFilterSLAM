# -*- coding: utf-8 -*-
"""
Created on Tue Aug 9 16:12:08 2016

Calculate the robot pose given the robot previous pose and motion of the robot with respect to that pose

@author: admin-u5941570
"""

import numpy as np

def Relative2AbsolutePose (robot_abs, u):
    
    '''
    Calculate the robot new pose given previous pose and motion
    Input: absolute coordinate of robot [x1,y1,theta1]
           motion command with respect to robot frame [dx, dy, dtheta]
    Output: absolute coordinate of robot next pose [x2,y2,theta2] 
    '''
    x1 = robot_abs[0][0]
    y1 = robot_abs[1][0]
    theta1 = robot_abs[2][0]
    dx = u[0][0]
    dy = u[1][0]
    dtheta = u[2][0]
    
    #R is the transition matrix of robot frame
    R = [[np.cos(theta1), -np.sin(theta1), 0],
         [np.sin(theta1), np.cos(theta1), 0],
         [0, 0, 1]]
         
    #Calculate Jacobian H1 with respect to X1
    H1 = [[1, 0, -dx*np.sin(theta1)-dy*np.cos(theta1)],
          [0, 1,  dx*np.cos(theta1)-dy*np.sin(theta1)],
          [0, 0, 1]]
         
    #Calculate Jacobian H2 with respect to u
    H2 = [[np.cos(theta1), -np.sin(theta1), 0],
          [np.sin(theta1), np.cos(theta1), 0],
          [0, 0, 1]]
     
    next_robot_abs = np.dot(R,u) + robot_abs
    
    return next_robot_abs, H1, H2
    
    
if __name__ == "__main__":
    
    '''
    For self testing only
    '''    
    
    robot_abs = [2,1,(1./4.)*(np.pi)]
    u = [0,2,0]
    next_robot_abs, H1, H2 = Relative2AbsolutePose(robot_abs,u)
    print (H1)
    
