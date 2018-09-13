# -*- coding: utf-8 -*-
"""
Created on Wed Aug 17 12:35:08 2016

Maps angle to the range of [-pi,pi]

@author: admin-u5941570
"""

import numpy as np

def pi2pi(angle):
        dp = 2*np.pi
        if (angle<=-dp) or (angle>=dp):
            angle = angle % dp
        if angle>=np.pi:
            angle = angle - dp
        if angle<=-np.pi:
            angle = angle + dp
        return angle
        
if __name__ == "__main__":
    
    '''
    For self testing only
    '''    
    ang = 5.13
    angle = pi2pi(ang)
    print (angle)
