#!/usr/bin/env python

import math
import numpy as np

class CaptureRegion:

	def __init__(self, objectRadius):
		self.objectRadius = objectRadius
		self.lengthHandCenter = 0.05 # 50 mm
		self.lengthBaseFinger = 0.07 # 70 mm
		self.lengthFingerEnd = 0.058 # 58 mm
		self.fingerBaseTipOffset = 0.003 # 3 mm
		self.minAperture = self.minAperture()

	def minAperture(self):

		# max object radius is 0.1557m
		# 87 degrees is the max handDegree when objectRadius = 0
		for handDegree in xrange(140):
			fingerDegree = handDegree + handDegree*45/140 + 40	# maybe 48/140 + 52
			handRadian = math.radians(handDegree)
			fingerRadian = math.radians(fingerDegree)
			if((self.lengthHandCenter + 
			    self.lengthBaseFinger * math.cos(handRadian) - 
			    self.fingerBaseTipOffset * math.sin(handRadian) + 
			    self.lengthFingerEnd * math.cos(fingerRadian)) < self.objectRadius):
				return math.radians(handDegree - 1)

	def initializeCaptureRegions(self, apertureAngles):
		self.apertureAngles = apertureAngles
		self.captureRegions = []

		for apertureAngle in apertureAngles:
			self.captureRegions.append(self.captureRegion(apertureAngle))


	def angleToFullHandWidth(self, handRadian):

		handDegree = math.degrees(handRadian)
		fingerDegree = handDegree + handDegree*45/140 + 40	# maybe 48/140 + 52
		fingerRadian = math.radians(fingerDegree)
		return 2*(self.lengthHandCenter + 
				  self.lengthBaseFinger * math.cos(handRadian) - 
				  self.fingerBaseTipOffset * math.sin(handRadian) + 
				  self.lengthFingerEnd * math.cos(fingerRadian))

	def captureRegion(self, handRadian):
		# handDegree is within 0 and 140 degrees
		# 0 degrees means the hand is fully open and 140 degrees means the hand is fully closed
		# The end finger degree can only go between 0 and 48 degrees and starts at an angle of 52 degrees to the finger base.
		# http://support.barrett.com/wiki/Hand/280/KinematicsJointRangesConversionFactors
		# http://www.barrett.com/images/HandDime4.gif

		handDegree = math.degrees(handRadian)
		fingerDegree = handDegree + handDegree*45/140 + 40	# maybe 48/140 + 52
		fingerRadian = math.radians(fingerDegree)

		rightBaseFingerStartX = self.lengthHandCenter;
		rightBaseFingerStartZ = 0;

		rightBaseFingerXDif = self.lengthBaseFinger * math.cos(handRadian)
		rightBaseFingerZDif = self.lengthBaseFinger * math.sin(handRadian)

		rightBaseFingerEndX = rightBaseFingerStartX + rightBaseFingerXDif
		rightBaseFingerEndZ  = rightBaseFingerStartZ + rightBaseFingerZDif

		rightFingerTipStartX = rightBaseFingerEndX - self.fingerBaseTipOffset * math.sin(handRadian)
		rightFingerTipStartZ = rightBaseFingerEndZ + self.fingerBaseTipOffset * math.cos(handRadian)

		rightFingerTipXDif = self.lengthFingerEnd * math.cos(fingerRadian)
		rightFingerTipZDif = self.lengthFingerEnd * math.sin(fingerRadian)

		rightFingerTipEndX = rightFingerTipStartX + rightFingerTipXDif
		rightFingerTipEndZ = rightFingerTipStartZ + rightFingerTipZDif

		# Using the distance of a point from a line formula here: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
		rightBaseFingerDesiredDist = -self.objectRadius * self.lengthBaseFinger
		rightFingerEndDesiredDist = -self.objectRadius * self.lengthFingerEnd

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

				if (zCoord > self.objectRadius):
					if ((zCoord > rightFingerTipEndZ) and (xCoord < rightFingerTipEndX - self.objectRadius)):
						array[zInd,xInd] = 1
					elif ((rightBaseFingerZDif * xCoord - rightBaseFingerXDif * zCoord + rightBaseFingerCrossProd < rightBaseFingerDesiredDist) and
				   		  (math.sqrt((rightFingerTipEndX - xCoord)**2 + (rightFingerTipEndZ - zCoord)**2) > self.objectRadius) and
				   		  (rightFingerTipZDif * xCoord - rightFingerTipXDif * zCoord + rightFingerEndCrossProd < 0) and
				   		  ((zCoord > rightFingerTipEndZ) or (rightFingerTipZDif * xCoord - rightFingerTipXDif * zCoord + rightFingerEndCrossProd < rightFingerEndDesiredDist))):
						array[zInd,xInd] = 1

		return (maxX, maxZ, array)


	def transformPoint(self, p, gSamples_i):
		transformationMatrix = np.zeros((3, 3))
		transformationMatrix[0,0] = math.cos(-p[2])
		transformationMatrix[1,1] = transformationMatrix[0,0]
		transformationMatrix[1,0] = math.sin(-p[2])
		transformationMatrix[0,1] = -transformationMatrix[1,0]
		transformationMatrix[0,2] = -(p[0] * transformationMatrix[0,0] + p[1] * transformationMatrix[0,1])
		transformationMatrix[1,2] = -(p[0] * transformationMatrix[1,0] + p[1] * transformationMatrix[1,1])
		transformationMatrix[2,2] = 1
		return transformationMatrix.dot(gSamples_i)

	def isInCaptureRegion(self, p, a, gSamples_i):

		#Use the capture region and check to see if the point is within the polygon of the capture region
		transformedPoint = self.transformPoint(p, gSamples_i)
		roundedTransformedPoint = np.around(transformedPoint, decimals=3)
		point_z = (roundedTransformedPoint[0])
		point_x = abs(roundedTransformedPoint[1])

		indexArray = np.where(self.apertureAngles == a)[0]
		if(indexArray.size > 0):	
			index = indexArray[0]
			(x_max, z_max, captureRegion) = self.captureRegions[index]
			
			if(point_x >= x_max or point_z >= z_max):
				return (False, -1)

			z_index = int(point_z * 1000)
			x_index = int(point_x * 1000)
			print (z_index, x_index)
			if(z_index < 0):
				return (False, -1)
			#import IPython
			#IPython.embed()
			return (captureRegion[z_index, x_index] == 1, max(0, 1 - (z_max - point_z)))

		else:
			return (False, -1)


if __name__ == "__main__":
	# Test cases
	cr = CaptureRegion(0.05)
	#print cr.minAperture
	#cr.initializeCaptureRegions(np.array([0, math.radians(5), math.radians(10), math.radians(15), math.radians(20), math.radians(25), math.radians(30)]))
	#print cr.isInCaptureRegion((-0.37156, -0.76, math.pi/2),0,(-0.37, -0.7, 1)) 
	# print cr.transformPoint((-0.37156, -0.73, math.pi/2),(-0.37, -0.7, 1)) 
	#print cr.minAperture
	#cr.initializeCaptureRegions(np.array([0, 5, 10, 15, 20, 25, 30]))
	#print cr.isInCaptureRegion([1,1,math.pi/4], 5, np.transpose(np.array([1.5,1.5,1])))
	#print cr.captureRegions

	#transform = isInCaptureRegion([1,1,45], 5, np.transpose(np.array([1.5,1.5,1])), 0)
	#print transform
	
	#maxDegree = minAperture(0.1)
	#print maxDegree
	#width = angleToFullHandWidth(0)
	#print width
	#(maxX, maxZ, captureregion) = captureRegion(0.01, 45)
	#np.savetxt("test.txt", captureregion, fmt='%d')