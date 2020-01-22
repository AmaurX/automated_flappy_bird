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

DISTANCE_TO_OBSTACLE_THRESHOLD = 0.50
ESPILON = 0.10
DEGREE_TO_RAD = 3.1415 / 180.0

class Planner():
    def __init__(self, state, obstacle):
        self.state = state
        self.obstacle = obstacle
        self.approach_angle = 0
        self.mode = "DISCOVERY"
    
    def plan(self):
        if (self.obstacle.x.estimate < DISTANCE_TO_OBSTACLE_THRESHOLD):
            if self.isAligned():
                self.approach_angle = 0.0
                self.mode = "PASSING_THROUGH"

            else:
                if self.state.y - self.obstacle.gapHeightFilter.estimate > 0.0:
                    self.approach_angle = -90.0 * DEGREE_TO_RAD
                else:
                    self.approach_angle =  90.0 * DEGREE_TO_RAD
                self.mode = "RECTIFICATION"
        else:
            x = self.obstacle.x.estimate - DISTANCE_TO_OBSTACLE_THRESHOLD
            y = self.obstacle.gapHeightFilter.estimate - self.state.y 
            R = np.linalg.norm([x,y])
            self.approach_angle = math.asin(y/R)
            self.mode = "APPROACH"
        
        print("approach angle is %.4f" % self.approach_angle)

    def isAligned(self):
        if abs(self.state.y - self.obstacle.gapHeightFilter.estimate) < ESPILON:
            return True
        return False
            