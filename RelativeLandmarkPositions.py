"""
Created on Mon Aug 29 13:36:08 2016

Calculate the relative positions of the landmarks

@author: admin-u5941570
"""

import numpy as np

def RelativeLandmarkPositions(landmark_abs, next_landmark_abs):
    
    '''
    Calculate the relative landmark positions
    Input: absolute coordinate of landmark [x1,y1]
           absolute coordinate of landmark next position [x2,y2]
    Output: relative position [dx, dy]
    '''
    # label is in position [0]
    x1 = float(landmark_abs[0])
    y1 = float(landmark_abs[1])
    x2 = float(next_landmark_abs[0])
    y2 = float(next_landmark_abs[1])
    
    #Calculate the difference of position in world frame
    diff = [[x2-x1], [y2-y1]]
    
    return diff
    
