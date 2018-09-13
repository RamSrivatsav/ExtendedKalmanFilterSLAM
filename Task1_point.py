#!/usr/bin/env python

'''
For publishing map
'''

import rospy
import time
import math
import tf
#import message_filters as mf

#from message_filters import TimeSynchronizer, Subscriber
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from Relative2AbsoluteXY_2 import Relative2AbsoluteXY 
from cylinder.msg import cylDataArray

x = 0
y = 0
th = 0
i_pose = 0
ty = 'POSE2D'



lx = 0
ly = 0
lax = 0
lay = 0
label = 0
ty2 = 'POINT2D'




def printPose(msg):
	print 'PoseCallback'
	global x
	global y
	global th
	global i_pose
	global lx
	global ly
	
	
	# Loading Values of Odometry.Pose
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	i_pose = msg.header.seq
	
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
	th = yaw   #theta = yaw?
	write2file()  
	
	
	

	

	
	
def cylinderCallBack(msg1):	
	print 'CylCallback'
	global x
	global y
	global th
	global i_pose
	global lx
	global ly
	global lax
	global lay
	global label
	robot_abs = [[0 for x in range (1)] for y in range (3)]
	robot_abs[0][0] = x
	robot_abs[1][0] = y
	robot_abs[2][0] = th
	
	landmark_meas_xy = [0 for x in range (2)]
       
    	if not msg1.cylinders==[]:
	    label = msg1.cylinders[0].label
	    landmark_meas_xy[0] = msg1.cylinders[0].Zrobot
	    landmark_meas_xy[1] = msg1.cylinders[0].Xrobot
	    cylPosX = msg1.cylinders[0].covariance[0]
	    cylPosDepth = msg1.cylinders[0].covariance[3]
	    
		##  @Mohit - Where is the id thing you said? I can't find it.
	    landmark_abs, H1, H2 = Relative2AbsoluteXY(robot_abs,landmark_meas_xy)
	    lax = landmark_abs[0]
	    lay = landmark_abs[1]
	else:
	    label = None
	    lax= None
	    lay = None
	    cylPosX = None
	    cylPosDepth = None
	
def write2file():
	

	global x
	global y
	global th
	global i_pose
	global lax
	global lay
	global label
	

	#adding to file
	
	p=open("pointDump1.txt","a+")
	
	p.write("%s \t %s \t %s \t %s \t %s \n"%(ty,i_pose,x,y,th))
	p.write("%s \t %s \t %s \t %s \t %s \n"%(ty2,label,lax,lay,i_pose))
	p.close()


def SyncCallback(odom,cyl):
	print 'SyncCallback'
	global x
	global y
	global th
	global i_pose
	global lx
	global ly
	global lax
	global lay
	global label
	
	
	# Loading Values of Odometry.Pose
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	i_pose = msg.header.seq
	
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
	th = yaw   #theta = yaw?
	
	
	robot_abs = [[0 for x in range (1)] for y in range (3)]
	robot_abs[0][0] = x
	robot_abs[1][0] = y
	robot_abs[2][0] = th
	
	landmark_meas_xy = [0 for x in range (2)]
       
    	if not msg1.cylinders==[]:
	    label = msg1.cylinders[0].label
	    '''
	    xc = msg1.cylinders[0].Zrobot * np.cos(msg1.cylinders[0].Xrobot)
	    yc = xc = msg1.cylinders[0].Zrobot * np.sin(msg1.cylinders[0].Xrobot)
	    landmark_meas_xy[0] = xc
	    landmark_meas_xy[1] = yc
	    '''
	    landmark_meas_xy[0] = msg1.cylinders[0].Zrobot
	    landmark_meas_xy[1] = -msg1.cylinders[0].Xrobot  #-ve casue  in slam students
	    
	    cylPosX = msg1.cylinders[0].covariance[0]
	    cylPosDepth = msg1.cylinders[0].covariance[3]
	    landmark_abs, H1, H2 = Relative2AbsoluteXY(robot_abs,landmark_meas_xy)
	    lax = landmark_abs[0]
	    lay = landmark_abs[1]
	else:
	    label = None
	    lax= None
	    lay = None
	    cylPosX = None
	    cylPosDepth = None
	    
	#adding to file
	p=open("pointDump1.txt","a+")
	
	p.write("%s \t %s \t %s \t %s \t %s \n"%(ty,i_pose,x,y,th))
	p.write("%s \t %s \t %s \t %s \t %s \n"%(ty2,label,lax,lay,i_pose))
	p.close()
	
	
	
### ENTRY POINT ###
if __name__ == '__main__': 
	
	
	rospy.init_node('Task1_point')
	#pub = rospy.Publisher('Task1_point', Odometry, queue_size=1)
	
	# Time Synchronizer
	#tSync=mf.TimeSynchronizer((mf.Subscriber("odom",Odometry),
	#					mf.Subscriber("/cylinderTopic",cylDataArray)),queue_size=1)
	#tSync.registerCallback(SyncCallback)
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
