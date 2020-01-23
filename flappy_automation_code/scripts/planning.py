#!/usr/bin/env python
import numpy as np
import math

DISTANCE_TO_OBSTACLE_THRESHOLD = 1.0
ESPILON = 0.05
DEGREE_TO_RAD = 3.14159265 / 180.0


class Planner():
    ''' 
    The role of the planner is to determine a single value : the approach angle

    The approach angle is then tracked by the controller (using Vx and Vy)

    The planner can be in different modes, that are linked by different possible transitions:

    DISCOVERY : initial state, before any obstacle is even detected
    APPROACH : Flappy is far away from the obstacle. 
            The current estimate of the height of the gap is used to generate a goal point a bit before the actual gap.
            This goal point is then used to determine is approach angle.
    PASSING_SOON : Flappy is closer to the obstacle, and checks if it is aligned with the gap
                If it is, the approach angle is 0.0 and Flappy should go foward
                If it is not, the mode becomes RECTIFICATION
    RECTIFICATION : If during PASSING_SOON, Flappy is not aligned with the gap, then the angle become +/- 90 degrees (meaning that Vx = 0),
                    to explore vertically the obstacle and align with the gap
    IS_PASSING : Once Flappy is very close to the obstacle, it is commited to pass no matter what. 
                When in mode PASSING_SOON or RECTIFICATION Flappy gets too close, then the angle becomes 0 degrees and it goes forward
                This serves to avoid weird behavior during passage if the perception suddendly changes its mind about the height of the gap
    '''

    def __init__(self, state, obstacle, ceiling_height):
        self.state = state
        self.obstacle = obstacle
        self.approach_angle = 0
        self.ceiling_height = ceiling_height
        self.mode = "DISCOVERY"

    def plan(self):
        if (self.obstacle.x.estimate < DISTANCE_TO_OBSTACLE_THRESHOLD):
            # Flappy is now close to the obstacle
            if self.isAligned():
                self.approach_angle = 0.0
                if self.obstacle.x.estimate < DISTANCE_TO_OBSTACLE_THRESHOLD/4.0:
                    self.mode = "IS_PASSING"
                else:
                    self.mode = "PASSING_SOON"

            else:
                if self.obstacle.x.estimate < DISTANCE_TO_OBSTACLE_THRESHOLD/5.0:
                    # When Flappy is too close to the obstacle, even if it doesn't believe it is aligned,
                    # the best strategy remains to go forward
                    self.mode = "IS_PASSING"
                if self.mode == "IS_PASSING":
                    self.approach_angle = 0.0

                else:
                    # Flappy is close but not aligned. It must enter into a rectification manoeuvre and move vertically only
                    if self.state.y - self.obstacle.gapHeightFilter.estimate > 0.0:
                        self.approach_angle = -90.0 * DEGREE_TO_RAD
                    else:
                        self.approach_angle = 90.0 * DEGREE_TO_RAD
                    self.mode = "RECTIFICATION"

        else:
            # Flappy is far from the obstacle and can do a soft approach manoeuvre
            x = self.obstacle.x.estimate - DISTANCE_TO_OBSTACLE_THRESHOLD
            y = self.obstacle.gapHeightFilter.estimate - self.state.y
            distanceToGoalPoint = np.linalg.norm([x, y])
            self.approach_angle = math.asin(y/distanceToGoalPoint)
            self.mode = "APPROACH"

    def isAligned(self):
        if abs(self.state.y - self.obstacle.gapHeightFilter.estimate) < ESPILON:
            return True
        return False
