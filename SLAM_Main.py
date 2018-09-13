#!/usr/bin/env python2

import rospy
import numpy as np
import unittest
import time as t
from collections import defaultdict
from scipy.linalg import block_diag
#Message types
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensors_msg.msg import Imu
from cylinder.msg import cylDataArray
from cylinder.msg import cylMsg
#Functions
from Relative2AbsolutePose import Relative2AbsolutePose
from Relative2AbsoluteXY import Relative2AbsoluteXY 
from Absolute2RelativeXY import Absolute2RelativeXY
from pi2pi import pi2pi
from mapping import mapping

#landmarks' most recent absolute coordinate     
landmark_abs_ = defaultdict(list)
seenLandmarks_ =[]
#State Transition Model
F_ = []
#Control-Input Model
W_ = []
# dimension of robot pose
dimR_ = 3

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
        
        self.posCov = new_Cov
    
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
        currentPose=self.robot.getPose(self.robot)
        currentCov=self.robot.getCovariance(self.robot)
        # TODO move robot given current pose and u
        # move command here?? publish()??
        
        # predict state mean
        [nextRobotPose,Hx,Hu]= Relative2AbsolutePose(currentPose, motion.getMotionCommand()) # done nextRobotAbs
        # predict state covariance
        nextCov = np.array(np.dot(np.dot(Hx, currentCov),np.transpose(Hx))) + np.array(np.dot(np.dot(Hu, motionCovariance),np.transpose(Hu)))
        # set robot new pose
        self.robot.setPose(self.robot,nextRobotPose)
        # set robot new covariance
        self.robot.setCovariance(self.robot,nextCov)
        # set KF priorStateMean
        priorStateMean=nextRobotPose
        # set KF priorStateCovariance
        priorStateCovariance=nextCov
        
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
        # if new landmark augment stateMean and stateCovariance
        if new:
            stateMean = np.concatenate((stateMean,[[landmarkAbs[0]], [landmarkAbs[1]]]),axis = 0)
            Prr = self.robot.getCovariance()  
            if len(seenLandmarks_) == 1:              
                Plx = np.dot(G1,Prr)
            else:
                lastStateCovariance    = KalmanFilter.getStateCovariance(self)
                Prm    = lastStateCovariance[0:3,3:]
                Plx    = np.dot(G1, np.bmat([[Prr, Prm]]))
            Pll = np.array(np.dot(np.dot(G1, Prr),np.transpose(G1))) + np.array(np.dot(np.dot(G2, measurementCovariance),np.transpose(G2)))
            P = np.bmat([[stateCovariance, np.transpose(Plx)],[Plx,Pll]])
            stateCovariance = P
        # else:
        # if old landmark stateMean & stateCovariance remain the same (will be changed in the update phase by the kalman gain)
        # calculate expected measurement
        [landmarkAbs Hr Hl] = Relative2AbsoluteXY(currentRobotAbs,measurement)
        # get measurement
        Z = ([ [measurement[0]],[measurement[1]] ])

        
        #Update
        x = stateMean
        label = measurement[2]
        
        # y = Z - expectedMeasurement
        y = Z - landmarkAbs
        
        # build H
        # H = [Hr, 0, ..., 0, Hl,  0, ..,0] position of Hl depends on when was the landmark seen? H is C ??
        H = np.reshape(Hr, (2, 3))
        for i in range(0, seenLandmarks_.index(label)):
            H = np.bmat([[H, np.zeros([2,2])]])
        H = np.bmat([[H, np.reshape(Hl, (2, 2))]])
        for i in range (0, len(seenLandmarks_)- seenLandmarks_.index(label)-1):
            H = np.bmat([[H, np.zeros([2,2])]])
            
        # compute S
        lastCovariance = self.robot.getCovariance()
        S =  np.array( np.dot( np.dot(H,lastCovariance),np.transpose(H) ) + measurementCovariance)
        
        if (S < 0.000001).all():
            print('Non-invertible S Matrix')
            raise ValueError
            return
        else:
        
        # calculate Kalman gain
        K=np.array(np.dot(np.dot(lastCovariance,np.transpose(H)),np.inverse(S)))			        
        # compute posterior mean
        posteriorStateMean = stateMean + np.dot(K,y)
        
        # compute posterior covariance
        kc=np.array(np.dot(K,H))
        posteriorStateCovariance = np.dot((np.eye(kc.size())-kc),stateCovariance)
        
        # check theta robot is a valid theta in the range [-pi, pi]
        posteriorStateMean[2][0] = pi2pi(posteriorStateMean[2][0])
        # update robot pose
        robotPose=np.array([posteriorStateMean[0][0]],[posteriorStateMean[1][0]],[posteriorStateMean[2][0]])
        # set robot pose
        self.robot.setPose(robotPose)
        # updated robot covariance
        robotCovariance= posteriorStateCovariance[0:3,0:3]
        # set robot covariance
        self.robot.setCovariance(robotCovariance)
        # set posterior state mean
        KalmanFilter.setStateMean(self,posteriorStateMean)
        # set posterior state covariance
        KalmanFilter.setStateCovariance(self,posteriorStateCovariance)
        print 'robot absolute pose : ',robotAbs
        vec = mapping(seenLandmarks_.index(label)+1)
        landmark_abs_[int(label)-1].append([[stateMean[dimR_ + vec[0]-1][0]],[stateMean[dimR_ + vec[1]-1][0]]])
        for i in range(0,len(landmark_abs_)):
            print 'landmark absolute position : ',i+1,',',np.median(landmark_abs_[i],0)
        return posteriorStateMean, posteriorStateCovariance      
        
last_time = 0  # For calculating dt in SLAM class
  
class SLAM(LandmarkMeasurement, Motion, KalmanFilter):
        
    def callbackOdometryMotion(self, msg):
        
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
	   
	   state = []
	   state[0][0] = x
	   state[1][0] = y
	   state[2][0] = th      
	   
        # TODO You can choose to only rely on odometry or read a second sensor measurement
        
        # TODO compute dt = duration of the sensor measurement
        global last_time
    	   current_time=rospy.get_time()
    	   dt=current_time-last_time
    	   last_time=current_time
        
        
        # TODO compute command
        #From odom or self calculate???
        
        #From odom
        dx=msg.twist.twist.linear.x
        dy=msg.twist.twist.linear.y
	   dth=msg.twist.twist.angular.z
	   
	   u=[]
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
        
        rospy.Subscriber('/Odomerty',Odometry,callbackOdometryMotion)
        rospy.Subscriber('/cylinderTopic',cylDataArray,callbackLandmarkMeasurement)
        
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

