#!/usr/bin/env python
import numpy as np
from scipy.ndimage.filters import median_filter

'''
This file contains two filters:

- The Kalman1D, to track the horizonal position of the obstacle

- The GapTracker, to track the hieght of the gap in the obstacle

'''


class Kalman1D():
    '''
    This 1D simple Kalman filter is used to track the distance of the obstacle to Flappy
    While there is uncertainty on the measure of the distance (the laser point clouds hits at different places),
    there is however no uncertainty on the predict part, because the dynamics of the game are deterministic
    This is why this 1D Kalman filter is very simple
    '''

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
    '''
    This filter is used to track the height of the gap in the obstacle.

    We first discrtize the Y axis with a N elements array, the "voteArray", and with N being "discretizationFactor" below

    Then, we use a "voting" system, where each ray of the laser scan either contribute with hits, or with miss.

    HIT : When a laser ray hits, the corresponding index is computed.
        The hit adds a lot of points to the index, as well as fewer points to the direct neighbours of this index

    MISS : When a laser ray misses, the corresponding index is computed.
        The miss removes a lot of points to the index, as well as fewer points to the direct neighbours of this index

    Adding or removing points to neighbours seems to improve the quality of the estimation.
    The idea it to emulate the addition of gaussian bumps to a continuous function (instead of adding Diracs).

    UNSEEN : At the end of one complete scan (one call of the laserscan callback), all the untouched indices of the voteArray gain some points as well
            This ensures that if Flappy didn't scan a part of the obstacle, it won't believe too much that the gap is there.
            This is the role of the "unseen" array
    '''

    def __init__(self, discretizationFactor, ceiling_height):
        self.voteArray = np.zeros(discretizationFactor)
        self.addBorderConditions()

        self.ceiling_height = ceiling_height
        self.discretizationFactor = discretizationFactor
        self.unseen = np.ones(discretizationFactor)

        self.estimate = self.ceiling_height / 2.0

    def addBorderConditions(self):
        '''
        The gap is mostly never on the edges, and the edges are rarely observed, 
        so they start with a positif obstacle count
        '''
        self.voteArray[0] = 40
        self.voteArray[1] = 20
        self.voteArray[-2] = 20
        self.voteArray[-1] = 40

    def addHit(self, height, uncertainty):
        # computing index of height
        index = int(height/self.ceiling_height * self.discretizationFactor)

        if(index >= self.discretizationFactor):
            print("height is bigger than ceiling...")
        else:
            self.voteArray[index] += 12
            self.unseen[index] = 0
            if(index - 1 >= 0):
                self.voteArray[index-1] += 4
                self.unseen[index-1] = 0

            if(index + 1 < self.discretizationFactor):
                self.voteArray[index+1] += 4
                self.unseen[index+1] = 0

        # Update the current estimate
        self.updateGapHeightEstimate()

    def addMiss(self, height, uncertainty):
        # computing index of height
        index = int(height/self.ceiling_height * self.discretizationFactor)

        if(index >= self.discretizationFactor):
            print("height is bigger than ceiling...")
        else:
            self.voteArray[index] -= 10
            self.unseen[index] = 0

            if(index - 1 >= 0):
                self.voteArray[index-1] -= 1

            if(index + 1 < self.discretizationFactor):
                self.voteArray[index+1] -= 1

        # Update the current estimate
        self.updateGapHeightEstimate()

    def endOfPointCloud(self):
        '''
        Adds the penalty to all unseen indices of the voteArray
        The penalty is quite small compared to a HIT
        '''
        self.voteArray += 3 * self.unseen
        self.unseen = np.ones(self.discretizationFactor)

    def updateGapHeightEstimate(self):
        '''
        The obstacle has a lot of tiny gaps in which Flappy cannot pass.
        Nevertheless, these gaps can be seen, and might give false positives 
        if we were to just take the argmin index of the voting array.

        To remove these outliers from our vote array, we use a median filter on it, with size 5.
        The small gaps will never be covered by more than 3 indices, and thus will be filtered out 
        to higher values by their neighbours.

        On the contrary, the good gap is covered by more than 3 indices in the voteArray and thus
        will not be filtered out.

        (Note that in order for this to works, the discretizationFactor and the size of the filter need to be adjusted together)

        Once the median filter is applied however, the gap might have muplitple indices with the exact same value, meaning that taking
        the argmin of it might not give the middle index of the gap. So we take the median of the 4 indices that give the smallest values 
        of the array

        '''
        median_filter(self.voteArray, size=5, output=self.voteArray,
                      mode="constant", cval=np.max(self.voteArray))

        # Take the indices in the order that sorts the array by growing value
        indicesInOrder = np.argsort(self.voteArray)

        # Give the four indices that give the smallest values of the array
        minIndex = np.median(indicesInOrder[:4])

        # Update the estimate
        self.estimate = (minIndex + 0.5) * \
            self.ceiling_height / self.discretizationFactor
