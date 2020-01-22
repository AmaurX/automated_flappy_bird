#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import math
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from scipy.ndimage.filters import gaussian_filter1d
from scipy.ndimage.filters import median_filter
from matplotlib import cm

# This 1D simple Kalman filter is used to track the distance of the obstacle to flappy  
class Kalman1D():
    def __init__(self, initialGuess):
        self.K = 1.0
        self.P = 8.0
        self.estimate = initialGuess

    def update(self, measure, uncertainty):
        self.K = self.P/(self.P + uncertainty)
        self.estimate += self.K * (measure - self.estimate)
        self.P *= (1.0 - self.K)

    def deterministic_predict(self, change):
        self.estimate += change


# This filter is used to track the height of the gap in the obstacle
# It uses a "voting" system, where each ray of the laser scan either contribute with hits, or with miss
class GapTracker():
    def __init__(self, discretizationFactor, ceiling_height):
        self.voteArray = np.zeros(discretizationFactor)
        self.ceiling_height = ceiling_height
        self.estimate = self.ceiling_height / 2.0
        self.discretizationFactor = discretizationFactor

    def addHit(self, height, uncertainty):
        index = int(height/self.ceiling_height * self.discretizationFactor)
        if(index >= self.discretizationFactor):
            print("height is bigger than ceiling...")
        else:
            self.voteArray[index] += 10
            if(index -1 >= 0):
                self.voteArray[index-1] += 2
            if(index + 1 < self.discretizationFactor):
                self.voteArray[index+1] += 2
        self.voteArray[0] += 0
        self.voteArray[-1] +=0
        self.updateGapHeightEstimate()

    
    def addMiss(self, height, uncertainty):
        index = int(height/self.ceiling_height * self.discretizationFactor)
        if(index >= self.discretizationFactor):
            print("height is bigger than ceiling...")
        else:
            self.voteArray[index] -= 5
            if(index -1 >= 0):
                self.voteArray[index-1] -= 1
            if(index + 1 < self.discretizationFactor):
                self.voteArray[index+1] -= 1
        
        self.updateGapHeightEstimate()

    def updateGapHeightEstimate(self):
        # self.voteArray /= 0.05 * np.linalg.norm(self.voteArray)
        median_filter(self.voteArray, size=5, output=self.voteArray, mode="constant", cval=np.max(self.voteArray))
        
        minIndex = np.argmin(self.voteArray)
        self.findCenterOfGap()
        self.estimate = (minIndex + 0.5) * self.ceiling_height / self.discretizationFactor

    def findCenterOfGap(self):
        return True