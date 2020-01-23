#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import math
import matplotlib.pyplot as plt
from matplotlib import cm
from planning import Planner
from controller import Controller
from filters import GapTracker, Kalman1D


'''
This is the main file of the automation.

It contains three classes:

    - The State class, to track Flappy's state
    - The Obstacle class, to track the obstacles' states
    - The AutomatedFlappy class, that is the master class that handles the ROS interface and gives birth to everything else

'''

CEILING = 4.05
OBSTACLE_WIDTH = 0.10


class State():
    '''
    Very simple class to store and update the state of Flappy
    It only consists of y position and Vx and Vy velocities
    '''

    def __init__(self):
        self.y = 2.0
        self.vy = 0.0
        self.vx = 0.0

    def updateWithSpeedAndDt(self, speed, dt):
        self.y += dt*speed

    def setInitialState(self, height):
        self.y = height


class Obstacle():
    '''
    A class to represent the obstacles

    It contains:
        - a x position estimator, that uses a custom 1DKalman filter
        - a gap height estimator, that uses the GapTracker filter
    '''

    def __init__(self, name, discretizationFactor):
        self.x = Kalman1D(5.0)
        self.gapHeightFilter = GapTracker(discretizationFactor, CEILING)
        self.isSeen = False
        self.name = name

    def updateXPositionWithSpeedAndDt(self, speed, dt):
        if (self.isSeen):
            self.x.deterministic_predict(-dt*speed)

    def updateXPosition(self, position, uncertainty):
        self.x.update(position, uncertainty)

    def updateGapPosition(self, position, uncertainty, hit):
        if hit:
            self.gapHeightFilter.addHit(position, uncertainty)
        else:
            self.gapHeightFilter.addMiss(position, uncertainty)


class AutomatedFlappy():
    '''
    This is the main class.

    It handles the ROS communication with the game, and stores and manages the different parts of the code:
        - The perception and state estimation : for Flappy and the obstacles
        - The planning part
        - The control part
    '''

    def __init__(self):
        # Initialize Flappy's state
        self.state = State()
        self.hasSetInitialState = False

        # Create the two obstacles
        self.discretizationFactor = 50
        self.firstObstacle = Obstacle(
            "first_obstacle", self.discretizationFactor)
        self.secondObstacle = Obstacle(
            "second_obstacle", self.discretizationFactor)

        # Create the planner, that needs references to the state and the first obstacle
        self.planner = Planner(self.state, self.firstObstacle, CEILING)

        # Create the controller, that needs references to the planner and to the state
        self.controller = Controller(self.state, self.planner)

    def flyLikeTheWind(self):
        '''
        This is the launch function. It creates the node, the publisher and subscibers,
        and then it calls the plot function so that the user gets a visual of the perception output 
        '''

        # Here we initialize our node running the automation code
        rospy.init_node('flappy_automation_code', anonymous=True)

        # Subscribe to topics for velocity and laser scan from Flappy Bird game
        rospy.Subscriber("/flappy_vel", Vector3, self.velCallback)
        rospy.Subscriber("/flappy_laser_scan", LaserScan,
                         self.laserScanCallback)

        # Create the publisher for acceleration
        self.pub_acc_cmd = rospy.Publisher(
            '/flappy_acc', Vector3, queue_size=1)

        # For the remainder of the time, we plot the current state and obstacle perception
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.plotPerception()
            rate.sleep()

    def velCallback(self, msg):
        # We do the predict part of our filters, and update the states
        self.state.updateWithSpeedAndDt(msg.y, 1.0/30.0)
        self.state.vy = msg.y
        self.state.vx = msg.x
        self.firstObstacle.updateXPositionWithSpeedAndDt(msg.x, 1.0/30.0)
        self.secondObstacle.updateXPositionWithSpeedAndDt(msg.x, 1.0/30.0)

        # When the first obstacle has been passed successfully, we replace the data 
        # of the first by the data of the second, which becomes the first
        # This is how the planner switches back to APPROACH mode
        # A new second obstacle is ready to start estimating
        if self.firstObstacle.x.estimate < -0.25:
            self.firstObstacle.x = self.secondObstacle.x
            self.firstObstacle.gapHeightFilter = self.secondObstacle.gapHeightFilter
            self.firstObstacle.isSeen = self.secondObstacle.isSeen
            self.secondObstacle.x = Kalman1D(5.0)
            self.secondObstacle.gapHeightFilter = GapTracker(
                self.discretizationFactor, CEILING)
            self.secondObstacle.isSeen = False
        
        # Call the planner
        self.planner.plan()

        # Ask the controller for the acceleration commands
        (x, y) = self.controller.giveAcceleration()

        # Publish these commands
        self.pub_acc_cmd.publish(Vector3(x, y, 0))

    def laserScanCallback(self, msg):
        # Give the initial estimate using the bottom ray at the very beginning
        if not self.hasSetInitialState:
            length = msg.ranges[0]
            angle = msg.angle_min
            distanceToGround = math.fabs(math.sin(angle) * length)
            self.state.setInitialState(distanceToGround)
            self.hasSetInitialState = True
        
        # Then, sort the ray and use them to determine the obstacle distance and the height of the gap
        for i in range(len(msg.intensities)):
            hasHit = bool(msg.intensities[i])
            length = msg.ranges[i]
            angle = msg.angle_min + i * msg.angle_increment

            # Computing distances from Flappy to hit point for X and Y
            height = math.sin(angle) * length
            distance = math.cos(angle) * length

            if hasHit:
                if (height < 0.1 - self.state.y):
                    # Then it touches the ground so we ignore it
                    # print("ray %d touches the ground" % i)
                    pass

                elif (height > CEILING - self.state.y - 0.1):
                    # Then it touches the ceiling
                    # print("ray %d touches the ceiling" % i)
                    pass
                else:
                    # When it hits, it can be on the first or on the second obstacle
                    if distance < 0.5 + self.firstObstacle.x.estimate:
                        # Update 1st obstacle distance
                        self.firstObstacle.updateXPosition(
                            distance + OBSTACLE_WIDTH/2.0, 1.0)
                        self.firstObstacle.isSeen = True

                        # Update Gap tracker
                        heightOfTheHit = self.state.y + \
                            self.firstObstacle.x.estimate * math.tan(angle)
                        self.firstObstacle.updateGapPosition(
                            heightOfTheHit, 10 * self.firstObstacle.x.estimate * self.firstObstacle.x.estimate, True)
                    else:
                        # Update second obstacle distance
                        self.secondObstacle.updateXPosition(
                            distance + OBSTACLE_WIDTH/2.0, 2.0)
                        self.secondObstacle.isSeen = True

                        # Update Gap tracker
                        heightOfTheHit = self.state.y + \
                            self.secondObstacle.x.estimate * math.tan(angle)
                        self.secondObstacle.updateGapPosition(
                            heightOfTheHit, 10 * self.secondObstacle.x.estimate * self.secondObstacle.x.estimate, True)
            else:
                # If a ray misses, we first need to decide if it actually has crossed an obstacle
                # So we compare the horizontal distance with the first obstacle distance
                if self.firstObstacle.isSeen and distance > self.firstObstacle.x.estimate:
                    # Then it sees through a gap
                    heightOfTheGap = self.state.y + \
                        self.firstObstacle.x.estimate * math.tan(angle)
                    self.firstObstacle.updateGapPosition(
                        heightOfTheGap, 10 * self.firstObstacle.x.estimate * self.firstObstacle.x.estimate, False)
                
                # And we then see if it had enough range to even go through the second obstacle
                if self.secondObstacle.isSeen and distance > self.secondObstacle.x.estimate:
                    # Then it sees through a gap
                    heightOfTheGap = self.state.y + \
                        self.secondObstacle.x.estimate * math.tan(angle)
                    self.secondObstacle.updateGapPosition(
                        heightOfTheGap, 10 * self.secondObstacle.x.estimate * self.secondObstacle.x.estimate, False)
            
            # At the end of the pointcloud parsing, we signal the gap filters. See filters.py for more info
            self.firstObstacle.gapHeightFilter.endOfPointCloud()
            self.secondObstacle.gapHeightFilter.endOfPointCloud()

    def plotPerception(self):
        # Let plt do dynamic plots
        plt.ion()
        # Clear current frame
        plt.clf()

        # Put the title
        plt.title('Title: {}'.format(self.planner.mode))
        plt.ylabel('height (in m)')
        plt.xlabel('distance (in m)')

        # Set the axes limits
        axes = plt.gca()
        axes.set_xlim([-0.4, 4.0])
        axes.set_ylim([-0.1, 4.2])

        # For each obstacle, plot a gray scale of the voteArray. 
        # The gap corresponds to the white, and the obstacle to the black
        for obstacle in [self.firstObstacle, self.secondObstacle]:
            xObstacle = obstacle.x.estimate * \
                np.ones(self.discretizationFactor)
            yObstacle = [
                (i + 0.5) * CEILING/self.discretizationFactor for i in range(self.discretizationFactor)]
            mini, maxi = np.min(obstacle.gapHeightFilter.voteArray), np.max(
                obstacle.gapHeightFilter.voteArray)
            diff = maxi - mini
            new_array = obstacle.gapHeightFilter.voteArray - mini
            if diff != 0:
                new_array /= float(diff)

            plt.scatter(xObstacle, yObstacle,
                        c=cm.gist_yarg(new_array), edgecolor='none', label="obstacle")

        # Plot in green the current best estimation of the gaps positions
        xGaps = [self.firstObstacle.x.estimate,
                  self.secondObstacle.x.estimate]
        yGaps = [self.firstObstacle.gapHeightFilter.estimate,
                  self.secondObstacle.gapHeightFilter.estimate]
        plt.plot(xGaps, yGaps, "go", label="Gaps")

        # Plot the position of Flappy in blue
        xFlappy = [0.0]
        yFlappy = [self.state.y] 
        plt.plot(xFlappy, yFlappy, "bo", label="Flappy")

        plt.show(block=False)
        plt.pause(0.01)


if __name__ == '__main__':
    try:
        automatedFlappy = AutomatedFlappy()
        automatedFlappy.flyLikeTheWind()
    except rospy.ROSInterruptException:
        pass
