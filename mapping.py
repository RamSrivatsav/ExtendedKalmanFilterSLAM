# -*- coding: utf-8 -*-
"""
Created on Wed Aug 17 12:35:08 2016

@author: admin-u5941570
"""

'''
Maps the position of the landmark in the stateMean vector
depending on its order in the list of seen landmarks
e.g  if cylinder 2 was the first cylinder to be seen 
then cylinder 1 was seen after cylinder 2 and before cylinder 5
seenLandmarks_ = [2,1,5]
# to find the index of landmark 5 in stateMean vector
# find position of label 5 in seenLandmarks_
label = 5
i = seenLandmarks_.index(label)+1    #i = 3
mapping(i) = [5,6] which is position of landmark with label 5 in the stateMean 
after the robot and starting from count 1
so you need to always add the dimension of the robot (3) and deduct 1 (python count starts from 0)
to get the exact position of landamrk in stateMean vector

[5,6]+ 3 -1 = [7,8] index of landmark 5 in stateMean

stateMean Vector looks like:
stateMean = 
[ rx;
 ry;
rtheta;
l2x;
l2y;
l1x;
l1y;
l5x;
l5y ]
'''

def mapping(i):
    dim = 2; # dimension of each landmark
    vec = []  
    for i in range ((i-1)* dim + 1, ((i-1)* dim) + dim + 1):
        vec.append(i)   
    return vec

if __name__ == "__main__":
    
    '''
    For self testing only
    '''    
    i = 3;
    vec = mapping(i)
    print (vec)
