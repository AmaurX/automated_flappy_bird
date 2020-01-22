#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import math
import matplotlib.pyplot as plt
from std_msgs.msg import Float32

first_obstacle_distance = 0
second_obstacle_distance = 0
flappy_y = 0
first_obstacle_gap_height = 0
second_obstacle_gap_height = 0


def plot_all(line1):

    x_data = [0.0,
              first_obstacle_distance,
              second_obstacle_distance]
    y_data = [flappy_y, first_obstacle_gap_height, second_obstacle_gap_height]
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
    plt.plot(x_data, y_data, "bo")
    plt.show(block=False)
    plt.pause(0.01)
    return line1


def main():
                # Here we initialize our node running the automation code
    rospy.init_node('flappy_plotter', anonymous=True)

    # Subscribe to topics for velocity and laser scan from Flappy Bird game
    rospy.Subscriber("/flappy_first_obstacle_estimate",
                     Float32, first_obstacle_distance_callback)
    rospy.Subscriber("/flappy_second_obstacle_estimate",
                     Float32, second_obstacle_distance_callback)
    rospy.Subscriber("/flappy_first_obstacle_gap",
                     Float32, first_obstacle_gap_height_callback)
    rospy.Subscriber("/flappy_second_obstacle_gap",
                     Float32, second_obstacle_gap_height_callback)
    rospy.Subscriber("/flappy_y_position",
                     Float32, flappy_y_callback)
    # Ros spin to prevent program from exiting
    rate = rospy.Rate(30)

    line1 = []
    while not rospy.is_shutdown():
        print("stuf")
        line1 = plot_all(line1)
        rate.sleep()


def first_obstacle_distance_callback(msg):
    global first_obstacle_distance
    first_obstacle_distance = msg.data


def second_obstacle_distance_callback(msg):
    global second_obstacle_distance
    second_obstacle_distance = msg.data


def first_obstacle_gap_height_callback(msg):
    global first_obstacle_gap_height
    first_obstacle_gap_height = msg.data


def second_obstacle_gap_height_callback(msg):
    global second_obstacle_gap_height
    second_obstacle_gap_height = msg.data


def flappy_y_callback(msg):
    global flappy_y
    flappy_y = msg.data


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
