#!/usr/bin/env python2

import rospy
import numpy as np
import unittest
import time as t
import tf
from collections import defaultdict
#from scipy.linalg import block_diag
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
from Absolute2RelativeXY import Absolute2RelativeXY
from pi2pi import pi2pi
from mapping import mapping
from numpy import array

from geometry_msgs.msg import Quaternion

#landmarks' most recent absolute coordinate     
landmark_abs_ = defaultdict(list)
seenLandmarks_ =[]
#State Transition Model
F_ = []
#Control-Input Model
W_ = []
# dimension of robot pose
dimR_ = 3

last_time = 0  # For calculating dt in SLAM class

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
        # TODO move robot given current pose and u
        # move command here?? publish()??
        
        # predict state mean
        [nextRobotPose,Hx,Hu]= Relative2AbsolutePose(currentPose, motion) # done nextRobotAbs
        print 'RobotPose:', currentPose
        print 'nextRobotPose:', nextRobotPose
        # predict state covariance
        #print 'Hx:',Hx
        nextCov = np.add(np.array(np.dot(np.dot(Hx, currentCov),np.transpose(Hx))), np.array(np.dot(np.dot(Hu, motionCovariance),np.transpose(Hu))))
        for i in range (0,len(seenLandmarks_)):
        	Hx=block_diag(Hx,[1],[1])
        	Hu=block_diag(Hu,[1],[1])#add 0 ?? To be tested
        	motionCovariance=np.array(block_diag(motionCovariance,[0],[0]))
        	
        print 'motion Cov',motionCovariance.shape
        print 'Hx:',Hx.shape
        print 'Hu:',Hu.shape
        print 'CurrentStateCov:',currentStateCov.shape
        nextStateCov = np.array(np.add(np.array(np.dot(np.dot(Hx, currentStateCov),np.transpose(Hx))), np.array(np.dot(np.dot(Hu, motionCovariance),np.transpose(Hu)))))
        #print 'nextCov:', nextCov
        # set robot new pose
        self.robot.setPose(nextRobotPose)
        # set robot new covariance
        self.robot.setCovariance(nextCov)
        # set KF priorStateMean
        self.stateMean[0][0]=nextRobotPose[0][0]
        self.stateMean[1][0]=nextRobotPose[1][0]
        self.stateMean[2][0]=nextRobotPose[2][0]
        priorStateMean=self.stateMean
        # set KF priorStateCovariance
        self.stateCovariance=nextStateCov
        priorStateCovariance=self.stateCovariance
        
        #print 'prior mean : ', priorStateMean
        #print 'prior covar : ', priorStateCovariance
        print 'xxxxxxxxxxxxxxxxx'
        
        return priorStateMean, priorStateCovariance

        
    def update(self,measurement, measurementCovariance, new):
        global seenLandmarks_
        global dimR_
        # get robot current pose
        currentRobotAbs=self.robot.getPose();
        
        # get landmark absolute position estimate given current pose and measurement (robot.sense)
        [landmarkAbs, G1, G2] = self.robot.inverseSense(currentRobotAbs, measurement)
        # get KF state mean and covariance
        stateMean=self.stateMean
        stateCovariance= self.stateCovariance
        
        # print 'update mean: ', currentRobotAbs
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
			#print 'Robo Land Else start'
			
			# GOING FOR LUNCH.. Will be back Soon
			
			lastStateCovariance    = KalmanFilter.getStateCovariance(self)
			#print 'STEP 1'
			#print 'Last covar: ',lastStateCovariance
			#end=len(lastStateCovariance[0][:])
			end = lastStateCovariance.shape
			print 'end:',end[1]
			
			#Prm    = lastStateCovariance[0:3, -1*(end[1]-3):(end[1]-1)]
			Prm = lastStateCovariance[0:3,3:] 
			#'''
			#print 'STEP 2'
			#print 'G1 : ', G1
			#print 'Prr : ', Prr
			#print 'Prm : ', Prm
			#'''
			Plx    = np.dot(G1, np.bmat([[Prr, Prm]]))
			#print 'Robo Lanf Else stop'
		Pll = np.array(np.dot(np.dot(G1, Prr),np.transpose(G1))) + np.array(np.dot(np.dot(G2, measurementCovariance),np.transpose(G2)))
		P = np.bmat([[stateCovariance, np.transpose(Plx)],[Plx,Pll]])
		stateCovariance = P
		#print ' Stop'
        # else:
        # if old landmark stateMean & stateCovariance remain the same (will be changed in the update phase by the kalman gain)
        # calculate expected measurement
        
        print 'state covar : ', stateCovariance.shape
        #print 'inside update', currentRobotAbs
        
        [landmarkAbs, Hr, Hl] = Relative2AbsoluteXY(currentRobotAbs,measurement)
        print 'Label : ', measurement[2]
        print 'land z: ', measurement[0]
        print 'land th: ',measurement[1]
        print 'landmarkAbs: ',landmarkAbs 
        print 'robot absolute pose : ',currentRobotAbs
        # get measurement
        Z = ([ [measurement[0]],[measurement[1]] ])

        
        #Update
        x = stateMean
        label = measurement[2]
        
        # y = Z - expectedMeasurement
        # AKA Innovation Term
        measured = ([ [landmarkAbs[0]],[landmarkAbs[1]] ])
        y = np.subtract(Z,measured)
        
        #print 'z: ', Z
        print '________'
        #print 'meas : ',  measured
        #print 'xxx'
        #print 'y : ', y
        #print '________________'
       
       
        # build H
        # H = [Hr, 0, ..., 0, Hl,  0, ..,0] position of Hl depends on when was the landmark seen? H is C ??
        H = np.reshape(Hr, (2, 3))
        
        print ' H Start: ', seenLandmarks_.index(label)
        
        for i in range(0, seenLandmarks_.index(label)):
            H = np.bmat([[H, np.zeros([2,2])]])
        H = np.bmat([[H, np.reshape(Hl, (2, 2))]])
        for i in range (0, len(seenLandmarks_)- seenLandmarks_.index(label)-1):
            H = np.bmat([[H, np.zeros([2,2])]])
        #print 'H done'
        #print 'HHHHHHHHHHHHHHHHHHHHHHH'
        # compute S
        # print 'Before Getting Stuck'
        #print 'H : ', H.shape
        #print 'State covar: ',stateCovariance
        #print '___________ERROR Start_______________'
        print 'G1 : ', G1
        print 'G2 : ', G2
        print 'H : ', H
        try:
        	s1 = np.dot(H,stateCovariance)
        except ValueError:
        	print 'Value Error S1'
        	print 'H shape',H.shape
        	print 'State Cov',stateCovariance.shape
        	return
        # print 's1: ', s1
        #print 'xxxxxxxxxxxxxxxx'
        #print 'Done s1'
        
        try:
        	S =  np.add(np.dot(np.dot(H,stateCovariance),np.transpose(H)), measurementCovariance)
        except ValueError:
        	print('Value error S')
        	return
        	
        #print '__________ERROR ZONE CROSSED________________'
        #print 'Done s'
        
        if (S < 0.000001).all():
            print('Non-invertible S Matrix')
            raise ValueError
            return
        #else:
        # print 'mat invertible'
        # calculate Kalman gain
        K=np.array(np.dot(np.dot(stateCovariance,np.transpose(H)),np.linalg.inv(S)))	
        
        #print 'K gain Done',K
        		        
        # compute posterior mean
        posteriorStateMean = np.add(stateMean, np.dot(K,y))
        
        #print ' New mean state DONE'
        
        # compute posterior covariance
        kc=np.array(np.dot(K,H))
        kcShape = len(kc)
        #print 'Kc shape',kcShape
        
        posteriorStateCovariance = np.dot(np.subtract(np.eye(kcShape),kc),stateCovariance)
        
        #print ' New Covar Done'
        # print 'xxxxxxxxxxxxxxxxxxxxxxxx'
        
        # check theta robot is a valid theta in the range [-pi, pi]
        posteriorStateMean[2][0] = pi2pi(posteriorStateMean[2][0])
        
        #print 'pi2pi Done'
        # update robot pose
        
        #print 'post state mean: ', posteriorStateMean
        
        robotPose=([posteriorStateMean[0][0]],[posteriorStateMean[1][0]],[posteriorStateMean[2][0]])
        
        #print 'calculate robot pose done'
        # set robot pose
        self.robot.setPose(robotPose)
        #print 'update',robotPose
        # updated robot covariance
        robotCovariance= posteriorStateCovariance[0:3,0:3]
        #print 'updated Cov',robotCovariance
        # set robot covariance
        self.robot.setCovariance(robotCovariance)
        # set posterior state mean
        KalmanFilter.setStateMean(self,posteriorStateMean)
        # set posterior state covariance
        KalmanFilter.setStateCovariance(self,posteriorStateCovariance)
        #print 'robot absolute pose : ',robotPose
        vec = mapping(seenLandmarks_.index(label)+1)
        landmark_abs_[int(label)-1].append([[stateMean[dimR_ + vec[0]-1][0]],[stateMean[dimR_ + vec[1]-1][0]]])
        for i in range(0,len(landmark_abs_)):
            print 'landmark absolute position : ',i+1,',',np.median(landmark_abs_[i],0)
        
        print 'post mean: ', posteriorStateMean
        print 'post covar: ', posteriorStateCovariance
        
        print '____END______'
        return posteriorStateMean, posteriorStateCovariance      
        

  
class SLAM(LandmarkMeasurement, Motion, KalmanFilter):
        
    def callbackOdometryMotion(self, msg):
		global last_time
		# read msg received
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y

		#Theta part
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

		state = [[0 for x in range (1)] for y in range (3)]
		state[0][0] = x
		state[1][0] = y
		state[2][0] = th      

		# TODO You can choose to only rely on odometry or read a second sensor measurement

		# TODO compute dt = duration of the sensor measurement

		current_time=rospy.get_time()
		dt=current_time-last_time
		last_time=current_time


		# TODO compute command
		#From odom or self calculate???

		#From odom
		dx=msg.twist.twist.linear.x
		dy=msg.twist.twist.linear.y
		dth=msg.twist.twist.angular.z

		u=[[0 for x in range (1)] for y in range (3)]
		u[0][0] = dx
		u[1][0] = dy
		u[2][0] = dth        


		# set motion command
		self.motion.setMotionCommand(u)

		# get covariance from msg received
		covariance = msg.twist.covariance
		self.motion.setCovariance([[covariance[0],0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,covariance[35]]])
		poseCovariance = self.robot.getCovariance()

		# call KF to execute a prediction
		self.KF.predict(self.motion.getMotionCommand(), self.motion.getCovariance())
        
    def callbackLandmarkMeasurement(self, data):
        global seenLandmarks_
        for i in range(0,len(data.cylinders)):
            # read data received
            # aligning landmark measurement frame with robot frame
            dx = data.cylinders[i].Zrobot
            dy = -data.cylinders[i].Xrobot
            label = data.cylinders[i].label 
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
            self.landmarkMeasurement.setCovariance([[covariance[0],0.0],[0.0, covariance[3]]])        
            measurementLandmarkCovariance = self.landmarkMeasurement.getCovariance()
            # call KF to execute an update
            try:
                self.KF.update(measurement, measurementLandmarkCovariance, new)  
            except ValueError:
                return
                      
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
        
        rospy.Subscriber('odom',Odometry,self.callbackOdometryMotion)
        rospy.Subscriber('/cylinderTopic',cylDataArray,self.callbackLandmarkMeasurement)
        
        # rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, gyro_result, queue_size=1) # If using Gyro in the future
                
        rospy.spin()
        
if __name__ == '__main__':
    print('Landmark SLAM Started...')
    # Initialize the node and name it.
    rospy.init_node('listener')
    
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        slam = SLAM()
    except rospy.ROSInterruptException: pass

