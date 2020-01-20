#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)

SCALING = 0.01

class State():
	def __init__(self):
		self.y = SCALING * 512.0/2.0
	
	def updateWithSpeedAndDt(self,speed, dt):
	    self.y += dt*speed

class Obstacle():
    def __init__(self):
        self.x = 20.0
        self.isSeen = False
        
    def updateXPositionWithSpeedAndDt(self, speed, dt):
        if (self.isSeen):
            self.x -= dt*speed
   
  
class Controller():
    def __init__(self):
        self.state = State()
        self.firstObstacle = Obstacle()
        
    def initNode(self):
        # Here we initialize our node running the automation code
        rospy.init_node('flappy_automation_code', anonymous=True)

        # Subscribe to topics for velocity and laser scan from Flappy Bird game
        rospy.Subscriber("/flappy_vel", Vector3, self.velCallback)
        rospy.Subscriber("/flappy_laser_scan", LaserScan, self.laserScanCallback)

        # Ros spin to prevent program from exiting
        rospy.spin()

    def velCallback(self, msg):
        # msg has the format of geometry_msgs::Vector3
        # Example of publishing acceleration command on velocity velCallback
        self.state.updateWithSpeedAndDt(msg.y, 1.0/30.0)
        self.firstObstacle.updateXPositionWithSpeedAndDt(msg.x, 1.0/30.0)
        x = 0
        y = 0
        print "Y Position: {}".format(self.state.y)
        print "X position of first obstacle: {}".format(self.firstObstacle.x)
        pub_acc_cmd.publish(Vector3(x,y,0))

    def laserScanCallback(self, msg):
        # msg has the format of sensor_msgs::LaserScan
        # print laser angle and range
        pass
        # print "Laser range: {}, angle: {}".format(msg.ranges[0], msg.angle_min)

if __name__ == '__main__':
    try:
        controller = Controller()
        controller.initNode()
    except rospy.ROSInterruptException:
        pass
