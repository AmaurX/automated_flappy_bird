#!/usr/bin/env python
import math

DESIRED_VELOCITY_NORM = 2.1


class Controller():
    '''
    This controller tracks target velocity values : Vx and Vy

    They are computed from the current state, from the desired velocity norm, and from the desired direction angle given by the planner

    The desired velocity norm is a constant. However, when flappy is closing on the obstacle but not yet commited to pass (e.g. in IS_PASSING mode),
    we dampen the velocity target by a factor that grows the closer we get to the obstacle. This allows for better final approach when 
    the estimation of the gap height stabalizes a bit late.



    The controler is a very simple P controller, with different gain for each part.

    The Kpx controller is huge because thanks to the dynamics of the game, Flappy cannot go backwards.
    This mean that there can be no oscillation and therefore a very high gain is gonna acheive the goal faster.
    This also works because there is no uncertainty in the dynamics, the state is always known perfectly.

    For Kpy, there can be oscillation, so this gain is of "normal" magnitude.

    One could try implementing a PI or PID controller, but the game is simple enough that a simple P controller works well.
    '''

    def __init__(self, state, planner):
        self.state = state
        self.planner = planner
        self.Kpx = 10000000
        self.Kpy = 50

    def giveAcceleration(self):
        # Getting the target velocity and angle
        desiredAngle = self.planner.approach_angle
        desiredSpeed = DESIRED_VELOCITY_NORM

        # When flappy is closing on the obstacle but not yet commited to pass (e.g. in IS_PASSING mode),
        # we dampen the velocity target by a factor that grows the closer we get to the obstacle
        if self.planner.mode != "IS_PASSING" and self.planner.obstacle.x.estimate - 0.1 >= 0.0:
            desiredSpeed *= 1 - 0.4/(0.9+self.planner.obstacle.x.estimate)

        # Computing the desired Vx annd Vy, that are actually tracked
        desiredVx = math.cos(desiredAngle) * desiredSpeed
        desiredVy = math.sin(desiredAngle) * desiredSpeed

        # Computing the diff between desired velocity and current velocity
        diffvy = desiredVy - self.state.vy
        diffvx = desiredVx - self.state.vx

        # The controler is a very simple P controller, with different gain for each part
        return (self.Kpx * diffvx, self.Kpy * diffvy)
