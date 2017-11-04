#!/usr/bin/env python

import math
import numpy as np

def minAperture(objectRadius):
	lengthHandCenter = 0.05 # 50 mm
	lengthBaseFinger = 0.07 # 70 mm
	lengthFingerEnd = 0.058 # 58 mm
	fingerBaseTipOffset = 0.003 # 3 mm

	# max object radius is 0.1557m
	# 87 degrees is the max handDegree when objectRadius = 0
	for handDegree in xrange(140):
		fingerDegree = handDegree + handDegree*45/140 + 40	# maybe 48/140 + 52
		handRadian = math.radians(handDegree)
		fingerRadian = math.radians(fingerDegree)
		if(lengthHandCenter + lengthBaseFinger * math.cos(handRadian) - fingerBaseTipOffset * math.sin(handRadian) + lengthFingerEnd * math.cos(fingerRadian) < objectRadius):
			return handDegree - 1


def angleToFullHandWidth(handDegree):
	lengthHandCenter = 0.05 # 50 mm
	lengthBaseFinger = 0.07 # 70 mm
	lengthFingerEnd = 0.058 # 58 mm
	fingerBaseTipOffset = 0.003 # 3 mm

	fingerDegree = handDegree + handDegree*45/140 + 40	# maybe 48/140 + 52
	handRadian = math.radians(handDegree)
	fingerRadian = math.radians(fingerDegree)
	return 2*(lengthHandCenter + lengthBaseFinger * math.cos(handRadian) - fingerBaseTipOffset * math.sin(handRadian) + lengthFingerEnd * math.cos(fingerRadian))

def captureRegion(objectRadius, handDegree):
	# handDegree is within 0 and 140 degrees
	# 0 degrees means the hand is fully open and 140 degrees means the hand is fully closed
	# The end finger degree can only go between 0 and 48 degrees and starts at an angle of 52 degrees to the finger base.
	# http://support.barrett.com/wiki/Hand/280/KinematicsJointRangesConversionFactors
	# http://www.barrett.com/images/HandDime4.gif

	fingerDegree = handDegree + handDegree*45/140 + 40	# maybe 48/140 + 52
	handRadian = math.radians(handDegree)
	fingerRadian = math.radians(fingerDegree)

	lengthHandCenter = 0.05 # 50 mm
	lengthBaseFinger = 0.07 # 70 mm
	lengthFingerEnd = 0.058 # 58 mm
	fingerBaseTipOffset = 0.003 # 3 mm

	rightBaseFingerStartX = lengthHandCenter;
	rightBaseFingerStartZ = 0;

	rightBaseFingerXDif = lengthBaseFinger * math.cos(handRadian)
	rightBaseFingerZDif = lengthBaseFinger * math.sin(handRadian)

	rightBaseFingerEndX = rightBaseFingerStartX + rightBaseFingerXDif
	rightBaseFingerEndZ  = rightBaseFingerStartZ + rightBaseFingerZDif

	rightFingerTipStartX = rightBaseFingerEndX - fingerBaseTipOffset * math.sin(handRadian)
	rightFingerTipStartZ = rightBaseFingerEndZ + fingerBaseTipOffset * math.cos(handRadian)

	rightFingerTipXDif = lengthFingerEnd * math.cos(fingerRadian)
	rightFingerTipZDif = lengthFingerEnd * math.sin(fingerRadian)

	rightFingerTipEndX = rightFingerTipStartX + rightFingerTipXDif
	rightFingerTipEndZ = rightFingerTipStartZ + rightFingerTipZDif

	# Using the distance of a point from a line formula here: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
	rightBaseFingerDesiredDist = -objectRadius * lengthBaseFinger
	rightFingerEndDesiredDist = -objectRadius * lengthFingerEnd

	rightBaseFingerCrossProd = rightBaseFingerEndX * rightBaseFingerStartZ - rightBaseFingerEndZ * rightBaseFingerStartX
	rightFingerEndCrossProd =  rightFingerTipEndX * rightFingerTipStartZ - rightFingerTipEndZ * rightFingerTipStartX

	stepsize = 0.001 # 1 mm accuracy

	Dmax = 1.0 # Max pushing distance (Set to 1m as in the paper)
	maxZ = round(Dmax + rightFingerTipEndZ + stepsize, 3)
	maxX = round(max(rightFingerTipEndX + stepsize, rightFingerTipStartX + stepsize), 3)

	# The array will be symmetrical, so there is no need to replicate the negative x side
	numArrayRows = int(maxZ / stepsize)
	numArrayCols = int(maxX / stepsize)
	array = np.zeros((numArrayRows, numArrayCols))

	for xInd in xrange(numArrayCols):
		xCoord = xInd * stepsize
		for zInd in xrange(numArrayRows):
			zCoord = zInd * stepsize

			if (zCoord > objectRadius):
				if ((zCoord > rightFingerTipEndZ) and (xCoord < rightFingerTipEndX - objectRadius)):
					array[zInd,xInd] = 1
				elif ((rightBaseFingerZDif * xCoord - rightBaseFingerXDif * zCoord + rightBaseFingerCrossProd < rightBaseFingerDesiredDist) and
			   		  (math.sqrt((rightFingerTipEndX - xCoord)**2 + (rightFingerTipEndZ - zCoord)**2) > objectRadius) and
			   		  (rightFingerTipZDif * xCoord - rightFingerTipXDif * zCoord + rightFingerEndCrossProd < 0) and
			   		  ((zCoord > rightFingerTipEndZ) or (rightFingerTipZDif * xCoord - rightFingerTipXDif * zCoord + rightFingerEndCrossProd < rightFingerEndDesiredDist))):
					array[zInd,xInd] = 1

	return (maxX, maxZ, array)


#def IsInCaptureRegion(p, a, gSamples_i, c):

	#Use the capture region and check to see if the point is within the polygon of the capture region


#def PlanPushGrasp(goalObject, obstacleObjects):

if __name__ == "__main__":
	# Test cases
	#maxDegree = minAperture(0.1)
	#print maxDegree
	#width = angleToFullHandWidth(0)
	#print width
	#(maxX, maxZ, captureregion) = captureRegion(0.01, 45)
	#np.savetxt("test.txt", captureregion, fmt='%d')