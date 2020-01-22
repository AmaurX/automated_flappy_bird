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

class Controller():
    def __init__(self, state, planner):
        self.state = state
        self.planner = planner
    
    def giveAcceleration(self):
        desiredAngle = self.planner.approach_angle
        currentAngle = math.atan2(self.state.vy, self.state.vx)
        # currentSpeed = np.linalg.norm([self.state.vy, self.state.vx])
        currentSpeed = 1.8
        diff = desiredAngle - currentAngle

        desiredVx = math.cos(desiredAngle) * currentSpeed
        desiredVy = math.sin(desiredAngle) * currentSpeed
        diffvy = desiredVy - self.state.vy
        diffvx = desiredVx - self.state.vx
        if(self.planner.mode == "RECTIFICATION"):
            return (150* diffvx,150* diffvy)
        else:
            return (15* diffvx,15* diffvy)

