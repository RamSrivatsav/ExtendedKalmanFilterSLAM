#!/usr/bin/env python2

import rospy
import numpy as np
import unittest
import time as t
import tf
import message_filters
from collections import defaultdict
from collections import Counter
from scipy.linalg import block_diag
#Message types
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from cylinder.msg import cylDataArray
from cylinder.msg import cylMsg
#Functions
from Relative2AbsolutePose import Relative2AbsolutePose
from Relative2AbsoluteXY import Relative2AbsoluteXY 
from RelativeLandmarkPositions import RelativeLandmarkPositions
from Absolute2RelativeXY import Absolute2RelativeXY
from pi2pi import pi2pi
from mapping import mapping
from numpy import array
from matplotlib import pyplot

from geometry_msgs.msg import Quaternion

#landmarks' most recent absolute coordinate     
landmark_abs_ = defaultdict(list)
seenLandmarks_ =[]
seenLandmarksX_=defaultdict(list)
#State Transition Model
F_ = []
#Control-Input Model
W_ = []
# dimension of robot pose
dimR_ = 3
it=0
last_time = 0  # For calculating dt in SLAM class

#list of priors and time
prior_=defaultdict(list)
priorCov_=defaultdict(list)

updateExecuting=False
predictOnce=True
theta_global = 0

class Robot ():
    
    def __init__ (self, pose, pos_Cov, sense_Type):
        
        self.x = pose[0][0]
        self.y = pose[1][0]
        self.theta = pose[2][0]
        self.poseCovariance = pos_Cov
        self.senseType = sense_Type
        
    def setPose (self,new_pose):
        
        self.x = new_pose[0][0]
        self.y = new_pose[1][0]
        self.theta = new_pose[2][0]
    
    def getPose(self):
        
        return [[self.x], [self.y], [self.theta]]
    
    
    def setCovariance (self, new_Cov):
        
        self.poseCovariance = new_Cov
    
    def getCovariance (self):
        
        return self.poseCovariance 
    
    def move (self, robotCurrentAbs, u):
        
        [nextRobotAbs, H1, H2] = Relative2AbsolutePose(robotCurrentAbs,u)
        
        self.x = nextRobotAbs[0][0]
        self.y = nextRobotAbs[1][0]
        self.theta = nextRobotAbs[2][0]
        return nextRobotAbs, H1, H2
        
    def sense (self, robotCurrentAbs, landmarkAbs):
	   
	   if self.senseType == 'Vision':
	   	[measurement, H1, H2] = Absolute2RelativeXY(robotCurrentAbs, landmarkAbs)
		           
	   else:
	   	raise ValueError ('Unknown Measurement Type')
		  
	   return measurement, H1, H2     
        
    def inverseSense (self, robotCurrentAbs, measurement):
        
        if self.senseType == 'Vision':
            [landmarkAbs, H1, H2] = Relative2AbsoluteXY(robotCurrentAbs, measurement)
	                
        else:
            raise ValueError ('Unknown Measurement Type')
            
        return landmarkAbs, H1, H2
        
class LandmarkMeasurement ():
    
    def __init__ (self, meas_Cov):
        
        self.measurementCovariance = meas_Cov
    
    def setCovariance (self,new_Cov):
        
        self.measurementCovariance = new_Cov
    
    def getCovariance (self):
        
        return self.measurementCovariance
        
class Motion ():
    
    def __init__ (self, motion_command, motion_Cov):
        
        self.u = motion_command
        self.motionCovariance = motion_Cov
            
    def setCovariance (self, new_Cov):
        
        self.motionCovariance = new_Cov
    
    def getCovariance (self):
        
        return self.motionCovariance
    
    def setMotionCommand (self, motionCommand):
        
        self.u = motionCommand
    
    def getMotionCommand (self):
        
        return self.u
    
    
class KalmanFilter(Robot):

    def __init__ (self, mean, covariance, robot):
        
        self.stateMean = mean 
        self.stateCovariance = covariance 
        self.robot = robot

    def setStateMean (self, mean):
        
        self.stateMean = mean
    
    def getStateMean (self):
        
        return self.stateMean 
    
    def setStateCovariance (self, covariance):
        
        self.stateCovariance = covariance
    
    def getStateCovariance (self):
        
        return self.stateCovariance
         
    def predict (self,motion, motionCovariance):
        # get robot current pose
        currentPose=self.robot.getPose()
        currentCov=self.robot.getCovariance()
        currentStateCov=np.array(self.stateCovariance)
                
        # predict state mean
        [nextRobotPose,Hx,Hu]= Relative2AbsolutePose(currentPose, motion) # done nextRobotAbs
        
        # predict state covariance
        #print 'Hx:',Hx
        nextCov = np.add(np.array(np.dot(np.dot(Hx, currentCov),np.transpose(Hx))), np.array(np.dot(np.dot(Hu, motionCovariance),np.transpose(Hu))))
        for i in range (0,len(seenLandmarks_)):
        	Hx=block_diag(Hx,[1],[1])
        	Hu=block_diag(Hu,[1],[1])#add 0 ?? To be tested
        	motionCovariance=np.array(block_diag(motionCovariance,[0],[0]))
        	
        
        nextStateCov = np.array(np.add(np.array(np.dot(np.dot(Hx, currentStateCov),np.transpose(Hx))), np.array(np.dot(np.dot(Hu, motionCovariance),np.transpose(Hu)))))
        #print 'nextCov:', nextCov
        if np.absolute(nextRobotPose[0][0])>3.5 or np.absolute(nextRobotPose[1][0])>3.5:
        	nextRobotPose=currentPose
        	nextStateCov=currentStateCov
        	
        
        # set robot new pose
        self.robot.setPose(nextRobotPose)
        # set robot new covariance
        self.robot.setCovariance(nextCov)
        # set KF priorStateMean
        self.stateMean[0][0]=nextRobotPose[0][0]
        self.stateMean[1][0]=nextRobotPose[1][0]
        self.stateMean[2][0]=pi2pi(nextRobotPose[2][0])
        priorStateMean=self.stateMean
        # set KF priorStateCovariance
        self.stateCovariance=nextStateCov
        priorStateCovariance=self.stateCovariance
        
        
        
        return priorStateMean, priorStateCovariance
        
    def update(self,measurement, measurementCovariance, new):
        global seenLandmarks_
        global dimR_
        global seenLandmarksX_
        global it
        # get robot current pose
        currentRobotAbs=self.robot.getPose()
        currentRobotCov=self.robot.getCovariance()
        label = measurement[2]
        # get landmark absolute position estimate given current pose and measurement (robot.sense)
        [landmarkAbs, G1, G2] = self.robot.inverseSense(currentRobotAbs, measurement)
        # get KF state mean and covariance
        currentStateMean=stateMean=np.array(self.stateMean)
        currentStateCovariance=stateCovariance= np.array(self.stateCovariance)
        
        print '###############################'
        
        # if new landmark augment stateMean and stateCovariance	   
        if new:
		stateMean = np.concatenate((stateMean,[[landmarkAbs[0]], [landmarkAbs[1]]]),axis = 0)
		Prr = self.robot.getCovariance()  
		# print 'Prr:',Prr
       
		if len(seenLandmarks_) == 1:
			#print 'Robo lanf If start '           
			Plx = np.dot(G1,Prr)
			#print'Robot Land If stop'
		else:
			lastStateCovariance    = KalmanFilter.getStateCovariance(self)
			Prm = lastStateCovariance[0:3,3:] 
			Plx    = np.dot(G1, np.bmat([[Prr, Prm]]))
			
		Pll = np.array(np.dot(np.dot(G1, Prr),np.transpose(G1))) + np.array(np.dot(np.dot(G2, measurementCovariance),np.transpose(G2)))
		P = np.bmat([[stateCovariance, np.transpose(Plx)],[Plx,Pll]])
		stateCovariance = P 
		
        else:
		# if old landmark stateMean & stateCovariance remain the same (will be changed in the update phase by the kalman gain)
		# calculate expected measurement
		vec = mapping(seenLandmarks_.index(label)+1)
		expectedMeas=[0,0]
		expectedMeas[0]=np.around(stateMean[dimR_ + vec[0]-1][0],3)
		expectedMeas[1]=np.around(stateMean[dimR_ + vec[1]-1][0],3)
		
		[landmarkRelative,_,_]=Absolute2RelativeXY(currentRobotAbs,expectedMeas)
		#Z = ([ [np.around(landmarkAbs[0],3)],[np.around(landmarkAbs[1],3)] ])
		measured= ([ np.around(landmarkRelative[0][0],3),np.around(landmarkRelative[1][0],3)])

		# y = Z - expectedMeasurement
		# AKA Innovation Term
		#measured = ([ [np.around(expectedMeas[0],3)],[np.around(expectedMeas[1],3)] ])
		Z = ([ np.around(measurement[0],3),np.around(measurement[1],3) ])
		
		
		y = np.array(RelativeLandmarkPositions(Z,measured))
		
		# build H
		# H = [Hr, 0, ..., 0, Hl,  0, ..,0] position of Hl depends on when was the landmark seen? H is C ??
		H = np.reshape(G1, (2, 3))
		for i in range(0, seenLandmarks_.index(label)):
			H = np.bmat([[H, np.zeros([2,2])]])
		H = np.bmat([[H, np.reshape(G2, (2, 2))]])
		for i in range (0, len(seenLandmarks_)- seenLandmarks_.index(label)-1):
			H = np.bmat([[H, np.zeros([2,2])]])
		
		measurementCovariance=np.array(measurementCovariance)
		try:
			S =  np.array(np.add(np.dot(np.dot(H,stateCovariance),np.transpose(H)), measurementCovariance))
		except ValueError:
			print('Value error S')
			print 'H shape',H.shape
			print 'State Cov',stateCovariance.shape
			print 'measurement Cov', measurementCovariance.shape
			return

		if (S < 0.000001).all():
			print('Non-invertible S Matrix')
			raise ValueError
			return
		
		# calculate Kalman gain
		K=np.array(np.dot(np.dot(stateCovariance,np.transpose(H)),np.linalg.inv(S)))

		# compute posterior mean
		posteriorStateMean = np.array(np.add(stateMean, np.dot(K,y)))

		# compute posterior covariance
		kc=np.array(np.dot(K,H))
		kcShape = len(kc)

		posteriorStateCovariance = np.dot(np.subtract(np.eye(kcShape),kc),stateCovariance)

		# check theta robot is a valid theta in the range [-pi, pi]
		posteriorStateMean[2][0] = pi2pi(posteriorStateMean[2][0])

		# update robot pose

		robotPose=([posteriorStateMean[0][0]],[posteriorStateMean[1][0]],[posteriorStateMean[2][0]])
		robotCovariance= posteriorStateCovariance[0:3,0:3]
		
		# updated robot covariance
		if not (np.absolute(posteriorStateMean[0][0])>3.5 or np.absolute(posteriorStateMean[1][0])>3.5):
		   	stateMean=posteriorStateMean
			stateCovariance=posteriorStateCovariance
			# set robot pose
			self.robot.setPose(robotPose)
			# set robot covariance
			self.robot.setCovariance(robotCovariance)
			print 'IM DONEXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX'
			
		
        # set posterior state mean
        self.stateMean=stateMean
        # set posterior state covariance
        self.stateCovariance=stateCovariance
        
        print 'Robot Pose:',currentRobotAbs
        vec = mapping(seenLandmarks_.index(label)+1)
        landmark_abs_[int(label)-1].append([[np.around(stateMean[dimR_ + vec[0]-1][0],3)],[np.around(stateMean[dimR_ + vec[1]-1][0],3)]])
        seenLandmarksX_[int(label)-1].append(np.around(stateMean[dimR_ + vec[0]-1][0],3))
        for i in range(0,len(landmark_abs_)):
        	#count=Counter(seenLandmarksX_[i])
        	print 'landmark absolute position : ',i+1,',',np.median(landmark_abs_[i],0)#count.most_common(1)
        
        print '____END______'
        return stateMean, stateCovariance      
        

  
class SLAM(LandmarkMeasurement, Motion, KalmanFilter):
        
	def callbackOdometryMotion(self, msg):
		global last_time
		global updateExecuting
		global theta_global
		# TODO You can choose to only rely on odometry or read a second sensor measurement

		# compute dt = duration of the sensor measurement
		current_time=rospy.get_time()
		dt=current_time-last_time
		
		#Assuming that motion will remain constant while update is executing
		if not updateExecuting:
			last_time=current_time
		else:
			if predictOnce:
				last_time=current_time
			else:
				return
		
		# compute motion command
		dx=msg.twist.twist.linear.x*dt
		dy=0.0
		dth=msg.twist.twist.angular.z*dt
		
		dth = pi2pi(dth)
		theta_global = dth
		

		u=[[0 for x in range (1)] for y in range (3)]
		u[0][0] = dx
		u[1][0] = dy
		u[2][0] = dth
		# set motion command
		self.motion.setMotionCommand(u)
		# get covariance from msg received
		covariance = np.multiply(msg.twist.covariance,dt)
		self.motion.setCovariance([[covariance[0],0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,covariance[35]]])
		poseCovariance = self.robot.getCovariance()

		# call KF to execute a prediction
		self.KF.predict(self.motion.getMotionCommand(), self.motion.getCovariance())
		
	def callbackAllTogether(self,msgOdom,msgGyro,msgCyl):
		global last_time
		global updateExecuting
		global theta_global
		
		
	
	
	
	def callbackSyncedMotion(self, msgOdom,msgGyro):
		global last_time
		global updateExecuting
		global theta_global
		# You can choose to only rely on odometry or read a second sensor measurement

		# compute dt = duration of the sensor measurement
		current_time=rospy.get_time()
		dt=current_time-last_time
		
		#Assuming that motion will remain constant while update is executing
		if not updateExecuting:
			last_time=current_time
		else:
			if predictOnce:
				last_time=current_time
			else:
				return
		
		# compute motion command
		dx=msgOdom.twist.twist.linear.x*dt
		dy=0.0
		dth=msgGyro.angular_velocity.z*dt
		
		if dth > 0.005:
			dth = 0.0		
		#print dx
		#print dth
		
		dth = pi2pi(dth)
		theta_global = dth
		

		u=[[0 for x in range (1)] for y in range (3)]
		u[0][0] = dx
		u[1][0] = dy
		u[2][0] = dth        
		# set motion command
		self.motion.setMotionCommand(u)
		# get covariance from msg received
		odomCovariance = np.multiply(msgOdom.twist.covariance,1)
		gyroCovariance=np.multiply(msgGyro.angular_velocity_covariance,1)
		self.motion.setCovariance([[odomCovariance[0],0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,gyroCovariance[8]]])
		poseCovariance = self.robot.getCovariance()

		# call KF to execute a prediction
		self.KF.predict(self.motion.getMotionCommand(), self.motion.getCovariance())
		
	def callbackLandmarkMeasurement(self, data):
		global seenLandmarks_
		global updateExecuting
		global predictOnce
		global theta_global
		updateExecuting=True
		for i in range(0,len(data.cylinders)):
		  # read data received
		  # aligning landmark measurement frame with robot frame
		  dx = data.cylinders[i].Zrobot
		  dy = -data.cylinders[i].Xrobot
		  label = data.cylinders[i].label 
		  
		  # Check for spurious reading
		  if dx>3.5:
		  	print 'Bullshit Reading'
		  	return
		  
		  x = np.dot(dx,np.cos(np.add(pi2pi(dy),theta_global)))
		  y = np.dot(dx,np.sin(np.add(pi2pi(dy),theta_global)))
		  
		  # determine if landmark is seen for first time 
		  # or it's a measurement of a previously seen landamrk
		  new = 0
		  # if seenLandmarks_ is empty
		  if not seenLandmarks_:
			 new = 1
			 seenLandmarks_.append(label)
		  # if landmark was seen previously
		  elif label not in seenLandmarks_:
			 new = 1
			 seenLandmarks_.append(label)      
		  measurement = []
		  measurement.append(dx)
		  measurement.append(dy)
		  measurement.append(label)
		  
		  
		  # get covariance from data received
		  covariance = data.cylinders[i].covariance
		  #self.landmarkMeasurement.setCovariance([[covariance[0],0.0],[0.0, covariance[3]]])       
		  self.landmarkMeasurement.setCovariance([[0.000001,0.0],[0.0, 0.000001]]) 
		  measurementLandmarkCovariance = self.landmarkMeasurement.getCovariance()
		  # call KF to execute an update
		  try:
			 
			 self.KF.update(measurement,measurementLandmarkCovariance, new)  
		  except ValueError:
		  	updateExecuting=False
		  	return
		  updateExecuting=False
		  predictOnce=True
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):

        
		# initialise a robot pose and covariance
		# TODO Need to Verify
		robot_pose = [[0.0], [0.0], [0.0]]
		print(robot_pose)        
		robot_covariance = [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]
		# Initialise robot         
		self.robot = Robot(robot_pose, robot_covariance, 'Vision' )

		# Initialise motion
		motionCommand = [[0.0], [0.0], [0.0]] # To BE MODIFIED
		# initialise a motion command and covariance
		motionCovariance = [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]] # To BE MODIFIED

		self.motion = Motion(motionCommand, motionCovariance)


		# Initialise landmark measurement
		measurement=[[0],[0]]
		# initialise a measurement covariance
		measurementCovariance = [[0.0,0.0], [0.0,0.0]]

		self.landmarkMeasurement = LandmarkMeasurement(measurementCovariance)

		###### Initialise kalman filter ####

		# initialise a state mean and covariance

		state_mean = [[0.0], [0.0], [0.0]]
		state_covariance = [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]

		# initial state contains initial robot pose and covariance
		self.KF = KalmanFilter(state_mean, state_covariance, self.robot)

		# Subscribe to different topics and assign their respective callback functions

		#rospy.Subscriber('odom',Odometry,self.callbackOdometryMotion)
		#rospy.Subscriber('/cylinderTopic',cylDataArray,self.callbackLandmarkMeasurement)

		#rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, gyro_result, queue_size=1) # If using Gyro in the future
		cylSub=message_filters.Subscriber('/cylinderTopic',cylDataArray)
		imuSub=message_filters.Subscriber('/mobile_base/sensors/imu_data', Imu)
		odomSub=message_filters.Subscriber('odom',Odometry)

		ts=message_filters.TimeSynchronizer([odomSub,imuSub,cylSub],10)
		ts.registerCallback(self.callbackSyncedMotion)


		rospy.spin()
        
if __name__ == '__main__':
    print('Landmark SLAM Started...')
    # Initialize the node and name it.
    rospy.init_node('listener')
    
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        slam = SLAM()
    except rospy.ROSInterruptException: pass

