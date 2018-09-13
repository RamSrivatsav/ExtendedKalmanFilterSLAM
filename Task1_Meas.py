#!/usr/bin/env python

'''
For publishing map
'''

import rospy
import time
import math
import tf
import numpy as np


from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from Relative2AbsoluteXY_2 import Relative2AbsoluteXY 
from cylinder.msg import cylDataArray


##################################################################

#old variables
xo = 0
yo = 0
tho = 0
i_poseo = 0

#pose val
x = 0
y = 0
th = 0

i_pose = 0
ty = 'ODOMETRY_MEAS2D'
dx = 0
dy = 0
dth = 0

#co var values 

	#new values 
sxx2 = 0
sxy2 = 0
syy2 = 0
sxth2 = 0
syth2 = 0
sthth2 = 0

	#old values 
xxo = 0
xyo = 0
yyo = 0
xth0 = 0
yth0 = 0
thth0 = 0	
	

#pose maes ids 
id_ref = 0
id_end = 0


#landmark var
lx = 0
ly = 0
lax = 0
lay = 0
label = 0
ty2 = 'LANDMARK_MEAS2D'

#landmark old pos with label
#lo =  label [old_abs_x old_abs_y ]
lo = [[0 for x in range (2)] for y in range (16)]

slxx2 = 0
slxy2 = 0
slyy2 = 0

##############################################################################

def printPose(msg):

	global dx
	global dy
	global dth
	global i_pose
	global sxx2
	global syy2
	global sthth2
	global sxy2
	global sxth2
	global syth2
	global xo  #old variable
	global yo
	global tho
	global id_ref
	global id_end
	#for the Landmark
	global x
	global y
	global th
	
	#define arrays for co var
	xa = [0 for x in range (2)]
	ya = [0 for x in range (2)]
	tha = [0 for x in range (2)]
	c = [[0 for x in range (3)] for y in range (3)]
	
	
	# Loading Values of Odometry.Pose
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	i_pose = msg.header.seq
	id_ref = id_end
	id_end = i_pose
	

	
	#calc Theta
	quaternion = (
	    msg.pose.pose.orientation.x,
	    msg.pose.pose.orientation.y,
	    msg.pose.pose.orientation.z,
	    msg.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	th = yaw   
	
	
	
	#load values to covar
	
	xa[0] = xo
	xa[1] = x
	
	ya[0] = yo
	ya[1] = y
	
	tha[0] = tho
	tha[1] = th
	
	#print(xa)      #test purpose
	
	#generate Co Var matrix
	variable = np.stack([xa,ya,tha])
	c = np.cov(variable)
	#print(c)      #test purpose     
	
	#Won't we need to square?
	sxx2 = c[0][0]^2
	sxy2 = c[0][1]^2
	sxth2 = c[0][2]^2
	syy2 = c[1][1]^2
	syth2 = c[1][2]^2
	sthth2 = c[2][2]^2
	
	
	
	#calc differences 
	dx = x -xo
	dy = y - yo
	dth = th -tho
	
	
	#update old values 
	
	xo = x
	yo = y
	tho = th
	
	
	write2file()  
	
#############################################################################	

def cylinderCallBack(msg1):	
	global x
	global y
	global th
	global i_pose
	
	
	global lx
	global ly
	global lax
	global lay
	global label
	global lo
	global slxx
	global slxy
	global slyy
	
	
	robot_abs = [[0 for x in range (1)] for y in range (3)]
	robot_abs[0][0] = x
	robot_abs[1][0] = y
	robot_abs[2][0] = th
	
	
	#define arrays for co var
	laxa = [0 for x in range (2)]
	laya = [0 for x in range (2)]

	c = [[0 for x in range (2)] for y in range (2)]	
	
	landmark_meas_xy = [0 for x in range (2)]
       
    	if not msg1.cylinders==[]:
    	
    	#load values from cylinder dectector
	    label = msg1.cylinders[0].label
	    landmark_meas_xy[0] = msg1.cylinders[0].Zrobot
	    landmark_meas_xy[1] = msg1.cylinders[0].Xrobot
	    cylPosX = msg1.cylinders[0].covariance[0]     #not used
	    cylPosDepth = msg1.cylinders[0].covariance[3]   #not used
	    
	#use helper to find absolute pose
	    landmark_abs, H1, H2 = Relative2AbsoluteXY(robot_abs,landmark_meas_xy)
	    lax = landmark_abs[0]
	    lay = landmark_abs[1]
	    
	    #print(lo)       #test purpose 	
	    
	#get Co-Var data 
	    laxa[0] = lo[label-1][0]
	    laxa[1] = lax
	
	    laya[0] = lo[label-1][0]
	    laya[1] = lay
		    
	#generate Co var    
	    c = np.cov([[laxa],[laya]])
	    
	    slxx = c[0][0]
	    slyy = c[1][1]
	    slxy = c[0][1]
	    
	#update old values 
	    lo[label-1][0]	= lax
	    lo[label-1][1]	= lay
		    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	else:
	    label = None
	    slxx= None
	    slyy = None
	    slxy = None
	    lax = None
	    lay = None	
	
	
	

	
	

	    

	
def write2file():
	

	global dx
	global dy
	global dth
	global i_pose
	global sxx2
	global syy2
	global sthth2
	global sxy2
	global sxth2
	global syth2
	global xo  #old variable
	global yo
	global tho
	global id_ref
	global id_end
	
	#Landmark vars
	global slxx
	global slyy
	global slxy
	global lax
	global lay
	global label
	
	

	#adding to file
	
	p=open("measDump1.txt","a+")
	
	p.write("%s \t %s \t %s \t %s \t %s \t %s \t %s \t %s \t %s \t %s \t %s \t %s \n"%(ty,id_ref,id_end,dx,dy,dth,sxx2,sxy2,sxth2,syy2,syth2,sthth2))
	p.write("%s \t %s \t %s \t %s \t %s \t %s \t %s \t %s \n"%(ty2,id_end,label,lax,lay,slxx,slxy,slyy))
	p.close()

	
	
### ENTRY POINT ###
if __name__ == '__main__': 
	
	
	rospy.init_node('Task1_meas')
	#pub = rospy.Publisher('Task1_meas', Odometry, queue_size=1)
	#Add synchronizer
	


	
	#Odom Subscriber
	odomSubscriber= rospy.Subscriber('odom',
                                       Odometry,
                                       printPose,
                                       queue_size=1)
    	#Cylinder Topic subscriber
	rospy.Subscriber('/cylinderTopic',
					cylDataArray,
					cylinderCallBack)
	
	            
                                       
	try: 
		rospy.spin()
	except:
		rospy.loginfo("node terminated.")
