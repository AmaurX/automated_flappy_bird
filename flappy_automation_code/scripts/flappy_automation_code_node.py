#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import math
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from scipy.ndimage.filters import gaussian_filter
from matplotlib import cm


# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)

FPS = 30
SCREENWIDTH = 432
SCREENHEIGHT = 512

# scale pixels to meters
SCALING = 0.01
ACCXLIMIT = 3.0
ACCYLIMIT = 35.0
VELLIMIT = 10.0 / (SCALING*FPS)

CEILING = 3.90
OBSTACLE_WIDTH = 0.30

class State():
    def __init__(self):
        self.y = SCALING * SCREENHEIGHT/2.0
        self.yPublisher = rospy.Publisher("/flappy_y_position",
                                          Float32, queue_size=1)

    def updateWithSpeedAndDt(self, speed, dt):
        self.y += dt*speed
        y_msg = Float32()
        y_msg.data = self.y
        self.yPublisher.publish(y_msg)

    def setInitialState(self, state):
        self.y = state
        y_msg = Float32()
        y_msg.data = self.y
        self.yPublisher.publish(y_msg)


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

class GapTracker():
    def __init__(self, discretizationFactor):
        self.voteArray = np.zeros(discretizationFactor)
        self.estimate = CEILING / 2.0
        self.discretizationFactor = discretizationFactor

    def addHit(self, height, uncertainty):
        index = int(height/CEILING * self.discretizationFactor)
        if(index >= self.discretizationFactor):
            print("height is bigger than ceiling...")
        else:
            self.voteArray[index] += 4
            if(index -1 >= 0):
                self.voteArray[index-1] += 2
            if(index + 1 < self.discretizationFactor):
                self.voteArray[index+1] += 2
        self.updateGapHeightEstimate()

    
    def addMiss(self, height, uncertainty):
        index = int(height/CEILING * self.discretizationFactor)
        if(index >= self.discretizationFactor):
            print("height is bigger than ceiling...")
        else:
            self.voteArray[index] -= 3
            if(index -1 >= 0):
                self.voteArray[index-1] -= 1
            if(index + 1 < self.discretizationFactor):
                self.voteArray[index+1] -= 1
        self.updateGapHeightEstimate()

    def updateGapHeightEstimate(self):
        # blurred = gaussian_filter(a, sigma=1)
        minIndex = np.argmin(self.voteArray)
        self.estimate = (minIndex + 0.5) * CEILING / self.discretizationFactor

class Obstacle():
    def __init__(self, name, discretizationFactor):
        self.x = Kalman1D(5.0)
        self.gapHeightFilter = GapTracker(discretizationFactor)
        self.isSeen = False
        self.name = name
        self.x_publisher = rospy.Publisher(
            "/flappy_%s_estimate" % name, Float32, queue_size=10)
        self.gap_publisher = rospy.Publisher(
            "/flappy_%s_gap" % name, Float32, queue_size=10)

    def updateXPositionWithSpeedAndDt(self, speed, dt):
        if (self.isSeen):
            self.x.deterministic_predict(-dt*speed)
            x_msg = Float32()
            x_msg.data = self.x.estimate
            self.x_publisher.publish(x_msg)

    def updateGapPosition(self, position, uncertainty, hit):
        if hit:
            self.gapHeightFilter.addHit(position, uncertainty)
        else:
            self.gapHeightFilter.addMiss(position, uncertainty)

        gap_msg = Float32()
        gap_msg.data = self.gapHeightFilter.estimate
        self.gap_publisher.publish(gap_msg)
        # print("height of the gap is : %.4f with uncertainty %.4f" %
        #       (self.gapHeightFilter.estimate, self.gapHeightFilter.P))


class FlappyController():
    def __init__(self):
        self.state = State()
        self.discretizationFactor = 50
        self.firstObstacle = Obstacle("first_obstacle", self.discretizationFactor)
        self.secondObstacle = Obstacle("second_obstacle", self.discretizationFactor)
        self.hasSetInitialState = False

    def initNode(self):
        # Here we initialize our node running the automation code
        rospy.init_node('flappy_automation_code', anonymous=True)

        # Subscribe to topics for velocity and laser scan from Flappy Bird game
        rospy.Subscriber("/flappy_vel", Vector3, self.velCallback)
        rospy.Subscriber("/flappy_laser_scan", LaserScan,
                         self.laserScanCallback)

        # Ros spin to prevent program from exiting
        rate = rospy.Rate(30)

        line1 = []
        while not rospy.is_shutdown():
            # print("stuf")
            line1 = self.plot_all(line1)
            rate.sleep()

    def velCallback(self, msg):
        # msg has the format of geometry_msgs::Vector3
        # Example of publishing acceleration command on velocity velCallback
        self.state.updateWithSpeedAndDt(msg.y, 1.0/30.0)
        self.firstObstacle.updateXPositionWithSpeedAndDt(msg.x, 1.0/30.0)
        self.secondObstacle.updateXPositionWithSpeedAndDt(msg.x, 1.0/30.0)

        if self.firstObstacle.x.estimate < -0.2:
            self.firstObstacle.x = self.secondObstacle.x
            self.firstObstacle.gapHeightFilter = self.secondObstacle.gapHeightFilter
            self.firstObstacle.isSeen = self.secondObstacle.isSeen
            self.secondObstacle.x = Kalman1D(5.0)
            self.secondObstacle.gapHeightFilter = GapTracker(self.discretizationFactor)
            self.secondObstacle.isSeen = False

        x = 0
        y = 0
        # print "Y Position: {}".format(self.state.y)
        print "X position of first obstacle: {}".format(self.firstObstacle.x.estimate)
        pub_acc_cmd.publish(Vector3(x, y, 0))

    def laserScanCallback(self, msg):
        # msg has the format of sensor_msgs::LaserScan
        # print laser angle and range
        if not self.hasSetInitialState:
            length = msg.ranges[0]
            angle = msg.angle_min
            distanceToGround = math.fabs(math.sin(angle) * length)
            self.state.setInitialState(distanceToGround)
            self.hasSetInitialState = True
        for i in range(len(msg.intensities)):
            hasHit = bool(msg.intensities[i])
            length = msg.ranges[i]
            angle = msg.angle_min + i * msg.angle_increment

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
                    if distance < 0.5 + self.firstObstacle.x.estimate:
                        # Update obstacle distance
                        self.firstObstacle.x.update(distance + OBSTACLE_WIDTH/2.0, 0.15)
                        self.firstObstacle.isSeen = True
                        
                        # Update Gap tracker
                        heightOfTheHit = self.state.y + self.firstObstacle.x.estimate * math.tan(angle)
                        self.firstObstacle.updateGapPosition(
                            heightOfTheHit, 10 * self.firstObstacle.x.estimate * self.firstObstacle.x.estimate, True)
                    else:
                        self.secondObstacle.x.update(distance + OBSTACLE_WIDTH/2.0, 0.15)
                        self.secondObstacle.isSeen = True
                                                # Update Gap tracker
                        heightOfTheHit = self.state.y + self.secondObstacle.x.estimate * math.tan(angle)
                        self.secondObstacle.updateGapPosition(
                            heightOfTheHit, 10 * self.secondObstacle.x.estimate * self.secondObstacle.x.estimate, True)
            else:
                if self.firstObstacle.isSeen and distance > self.firstObstacle.x.estimate:
                    # Then it sees through a gap
                    heightOfTheGap = self.state.y + \
                        self.firstObstacle.x.estimate * math.tan(angle)
                    self.firstObstacle.updateGapPosition(
                        heightOfTheGap, 10 * self.firstObstacle.x.estimate * self.firstObstacle.x.estimate, False)
                    # print "Laser range: {}, angle: {}".format(msg.ranges[0], msg.angle_min)
                if self.secondObstacle.isSeen and distance > self.secondObstacle.x.estimate:
                    # Then it sees through a gap
                    heightOfTheGap = self.state.y + \
                        self.secondObstacle.x.estimate * math.tan(angle)
                    self.secondObstacle.updateGapPosition(
                        heightOfTheGap, 10 * self.secondObstacle.x.estimate * self.secondObstacle.x.estimate, False)
                    # print "Laser range: {}, angle: {}".format(msg.ranges[0], msg.angle_min)
    
    def plot_all(self, line1):
        x_data = [0.0,
                self.firstObstacle.x.estimate,
                self.secondObstacle.x.estimate]
        y_data = [self.state.y, self.firstObstacle.gapHeightFilter.estimate, self.secondObstacle.gapHeightFilter.estimate]
        # if line1 == []:
        plt.ion()
        # fig = plt.figure(figsize=(13, 6))
        # ax = fig.add_subplot(111)
        # # create a variable for the line so we can later update it
        # line1, = ax.plot(x_data, y_data, "bo")
        # # update plot label/title
        # # plt.ylabel('Y Label')
        # # plt.title('Title: {}'.format(identifier))
        # plt.show()

        # line1.set_xdata(x_data)
        # line1.set_ydata(y_data)
        plt.clf()
        axes = plt.gca()
        axes.set_xlim([-0.4, 6.0])
        axes.set_ylim([-0.1, 4.2])

        x_first_obstacle = self.firstObstacle.x.estimate * np.ones(self.discretizationFactor)
        y_first_obstacle = [(i+0.5) * CEILING/self.discretizationFactor for i in range(self.discretizationFactor)]
        
        
        mini, maxi = np.min(self.firstObstacle.gapHeightFilter.voteArray) , np.max(self.firstObstacle.gapHeightFilter.voteArray)
        diff = maxi - mini
        new_array = self.firstObstacle.gapHeightFilter.voteArray + mini
        new_array /= diff
        
        plt.scatter(x_first_obstacle,y_first_obstacle, c=cm.hot(new_array), edgecolor='none')
        
        plt.plot(x_data, y_data, "b.")
        plt.show(block=False)
        plt.pause(0.01)
        return line1

if __name__ == '__main__':
    try:
        flappyController = FlappyController()
        flappyController.initNode()
    except rospy.ROSInterruptException:
        pass
