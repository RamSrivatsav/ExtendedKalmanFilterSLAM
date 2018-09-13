import numpy as np
from RelativeLandmarkPositions import RelativeLandmarkPositions 
from pi2pi import pi2pi

'''
Calculate the error between GT data and SLAM obtained  data
Input: - solution file containing:
       - absolute coordinates of landmark positions
   - GT file containing:
       - GT absolute coordinates of landmark positions
Output: error = Relative GT landmark positions - Relative estimated landmark positions
'''

def ErrorFunction (solutionFile, GTFile):
    
     landmark = []
     landmark_GT = []
     if solutionFile.endswith('.csv') and GTFile.endswith('.csv'):
         with open(solutionFile, 'rb') as csvfile:
		 spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
		 for row in spamreader:
		    info = row[0].split(',')
		    if(info[0]=='POINT2D'):
		        landmark.append([info[1],info[2],info[3]])
                     
         with open(GTFile, 'rb') as csvfile:
		 spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
		 for row in spamreader:
		    info = row[0].split(',')
		    if(info[0]=='POINT2D'):
		        landmark_GT.append([info[1],info[2],info[3]])        
     
     elif solutionFile.endswith('.txt') and GTFile.endswith('.txt'):    
         f = open(solutionFile, 'r')
         for line in f:
            info = line.split(' ')
            if(info[0]=='POINT2D'):
                landmark.append([info[1],info[2],info[3]])
            
         f = open(GTFile, 'r')
         for line in f:
            info = line.split(' ')
            if(info[0]=='POINT2D'):
                landmark_GT.append([info[1],info[2],info[3]])   

     landmarkError = []
     landmark = sorted(landmark)
     for i in range (0,len(landmark)-1):
         RelLandmarks = RelativeLandmarkPositions(landmark[i],landmark[i+1])
         landmark1 = landmark[i]
         label1 = int(landmark1[0])
         landmark2 = landmark[i+1]
         label2 = int(landmark2[0])
         # GT file should be ordered by cylinders label 
         # e.g.
         # POINT2D 1 x1 y1
         # POINT2D 2 x2 y2
         # '
         # '
         # '
         RelLandmarksGT = RelativeLandmarkPositions(landmark_GT[label1-1],landmark_GT[label2-1])
         landmarkError.append(np.array(RelLandmarksGT) - np.array(RelLandmarks))
         
     errorLandmark = (1.0/(len(landmark)-1))*np.linalg.norm(landmarkError)
     
     print 'Your Solution Error', errorLandmark
     
     return errorLandmark
        
if __name__ == "__main__":
    
    '''
    For self testing only
    '''    
    solutionFile = 'turtlebotSolution.txt'
    GTFile = 'gt.txt'
    error = ErrorFunction(solutionFile, GTFile)
    

