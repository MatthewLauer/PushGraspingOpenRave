import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
from lineExample import handMover
import numpy
import IPython
import time
import math
from CaptureRegion import CaptureRegion

class PushStateMachine:

	def __init__(self, env):
	    self.handMover = handMover();
	    self.env = env;
	    self.robot = env.GetRobots()[0]
	    self.robot.SetActiveManipulator('arm')
	    self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=self.robot,iktype=IkParameterization.Type.Transform6D)
	    if not self.ikmodel.load():
	        self.ikmodel.autogenerate()
	    self.basemanip = interfaces.BaseManipulation(self.robot)


	#Takes an object in the scene and returns an array of tuples 
	# with suggested hand pose (x, y, theta)
	# kinboayd is the object you want 
	def GetPose(self, o, v, a, trans):
		dofs = self.robot.GetActiveDOFValues()
		dofs[7] = a;
		dofs[8] = a;
		dofs[9] = a;
		self.robot.SetActiveDOFValues(dofs)

		rotatedtrans = trans.copy()
		rotatedtrans[0:3,0:3] = numpy.dot(matrixFromAxisAngle([0,0,v])[0:3,0:3], rotatedtrans[0:3,0:3])
		rotatedtrans[1][3] = rotatedtrans[1][3] + math.sin(v)*o        			
		rotatedtrans[0][3] = rotatedtrans[0][3] + math.cos(v)*o        			
		
		sol = self.ikmodel.manip.FindIKSolution(rotatedtrans, IkFilterOptions.IgnoreEndEffectorCollisions)
		if sol is None:
			#print 'fail'
			return None
		#IPython.embed()
		armPoses = numpy.empty([0,11])
		while True:
			rotatedtrans[1][3] = rotatedtrans[1][3] + math.cos(v)*backwardincrement      			
			rotatedtrans[0][3] = rotatedtrans[0][3] - math.sin(v)*backwardincrement        			
			#IPython.embed()
			anysol = self.ikmodel.manip.FindIKSolution(rotatedtrans, IkFilterOptions.IgnoreEndEffectorCollisions)
			if anysol is None:
				#print 'fail'
				return None
			if anysol is not None:
				sol11DOF = numpy.append(anysol,self.robot.GetActiveDOFValues()[7:11])				
				self.robot.SetActiveDOFValues(sol11DOF)
				#time.sleep(.1)
			colsol = self.ikmodel.manip.FindIKSolution(rotatedtrans, IkFilterOptions.CheckEnvCollisions)
			if colsol is not None:
				sol11DOF = numpy.append(colsol,self.robot.GetActiveDOFValues()[7:11])				
				armPose = sol11DOF
				self.robot.SetActiveDOFValues(sol11DOF)
				#time.sleep(.1)
				#print 'success'
				break
		return armPose

	def sampleTransformXY(self, transform, sigma = .01):
		trans = transform.copy()
		trans[1][3] = numpy.random.normal(trans[1][3], sigma, 1)
		trans[0][3] = numpy.random.normal(trans[0][3], sigma, 1)
		return trans


	#Terrible approxiamation that is sufficient for now
	def _ApetureAngleToDistance(self,angle):
		return 2*.075*math.cos(angle)


	def MoveGripper(self, trans):
		try:
			print 'attempting to move hand'
			#IPython.embed()
			#translation = pose[4:7]
			#rotation = pose[0:4]
			#success = self.basemanip.MoveToHandPosition(translation = translation, rotation = rotation, execute=True,outputtraj=True,minimumgoalpaths=1)
			success = self.basemanip.MoveToHandPosition(matrices=[trans], seedik=10)
			robot.WaitForController(0)
			return success
		except planning_error,e:
		    return None

	def MoveGripperStraight(self, distance, Tee, dirAngle):
		try:
			#print 'attempting to move straight'
			if(distance ==  0):
				return True
			stepsize = .01
			steps = distance /stepsize
			direction = [math.sin(dirAngle),-math.cos(dirAngle), 0]
			#IPython.embed()
			success = self.basemanip.MoveHandStraight(direction=direction,starteematrix=Tee,stepsize=stepsize,minsteps=steps-1,maxsteps=steps,outputtraj = True)
			robot.WaitForController(0)

			return success
		except Exception as e:
			return None



if __name__ == "__main__":
	env = openravepy.Environment()
	env.Load('env.xml')
	env.SetViewer('qtcoin')
	PSM = PushStateMachine(env)
	goalRadius = .05;
	sigma = .005
	CR = CaptureRegion(goalRadius/2)
	#More Initialization							+
	bodySampleSize = 30

    #sample obstacles								+
	body = []
	with env:
		boxtrans = numpy.array([[1,0,0,-.6785],[0,0,-1,-.6],[0,1,.0,1.112], [0,0,0,1]])
		for i in range(bodySampleSize):
			tempbody = RaveCreateKinBody(env,'')
			tempbody.SetName(("SmallBox" + str(i)))
			tempbody.InitFromBoxes(numpy.array([[0,0,0,.1,0.05,0.075]]), True)
			trans = PSM.sampleTransformXY(boxtrans, sigma)
			if(i != 0):
				tempbody.SetTransform(trans)
			else:
				tempbody.SetTransform(boxtrans)
			body.append(tempbody)
			env.AddKinBody(body[i])

	body2 = []
	with env:
		boxtrans = numpy.array([[1,0,0,-.4785],[0,0,-.9,-.6],[0,1,.0,1.112], [0,0,0,1]])
		for i in range(bodySampleSize):
			tempbody = RaveCreateKinBody(env,'')
			tempbody.SetName(("LargeBox" + str(i)))
			tempbody.InitFromBoxes(numpy.array([[0,0,0,.1,0.05,0.175]]), True)
			trans = PSM.sampleTransformXY(boxtrans, sigma)
			if(i != 0):
				tempbody.SetTransform(trans)
			else:
				tempbody.SetTransform(boxtrans)
			body2.append(tempbody)
			env.AddKinBody(body2[i])

    #sample goal object 							+
	goal = []
	with env:
		goaltrans = numpy.array([[1,0,0,-.700],[0,0,-.9,-.37],[0,1,.0,1.112], [0,0,0,1]])
		for i in range(bodySampleSize):
			tempbody = RaveCreateKinBody(env,'')
			tempbody.SetName(("Goal" + str(i)))
			tempbody.InitFromSpheres(numpy.array([[0,0,0,goalRadius]]), True)
			trans = PSM.sampleTransformXY(goaltrans, sigma)
			if(i != 0):
				tempbody.SetTransform(trans)
			else:
				tempbody.SetTransform(goaltrans)
			goal.append(tempbody)
			env.AddKinBody(goal[i])

	viewer = env.GetViewer()
	viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
	robot = env.GetRobots()[0]
	dofs = [-2.0,1.0,2.0,-.5,-1.0,1.0,1.0,1.0,1.0,1.0,0.0]
	robot.SetActiveDOFValues(dofs)


	#Setup parameters for offsets to loop through
	minAperture = CR.minAperture
	vStepCount=15; oStepCount=1; aStepCount = 4; backwardincrement = .03

	trans = goal[0].GetTransform()
	sol = numpy.array([0,0,0,0,0,0,0])#7Dof #-(CR.angleToFullHandWidth(0)-CR.angleToFullHandWidth(minAperture))/2
	oSteps = numpy.linspace(0,
		(CR.angleToFullHandWidth(0)-CR.angleToFullHandWidth(minAperture))/2, num=oStepCount*2+1)
	vSteps = numpy.linspace(0,2*math.pi, num = vStepCount, endpoint = False)
	aSteps = numpy.linspace(0,minAperture,num = aStepCount)

	CR.initializeCaptureRegions(aSteps)
	#Loop through offset parameters a,v,o 	
	#time.sleep(5)
	data_file = open("Data.txt", "a")
	data_file.write("\nSamples: %f Sigma: %f Obstacles: %f" %(bodySampleSize, sigma, 2))

	import IPython
	push = None
	time.sleep(15)
	start = time.time()
	for o in oSteps:
		for v in vSteps:
			for a in aSteps:
				pose = PSM.GetPose(o,v,a,goaltrans) 
				if(pose is None):
					continue

				#print pose
				#print 'o: %f v: %f a: %f' % (o,v,a)
				robot.SetActiveDOFValues(pose)
				robottrans = robot.GetManipulator("arm").GetTransform()
				p = (robottrans[1][3], robottrans[0][3], v)
				#print "Hand Pose:"
				#print p
				#print "Goals Samples:"
				for goalSample in goal:
					goalSampleTrans = goalSample.GetTransform()
					gSamples = (goalSampleTrans[1][3], goalSampleTrans[0][3], 1)
					#print gSamples
					success = CR.isInCaptureRegion(p, a, gSamples)
					if success[0] == False:
						break

				if(success[0] == False):
					continue
				dofs[7:10] = [a,a,a]
				robot.SetActiveDOFValues(dofs)
				PSM.MoveGripper(robottrans)
				for goalSample in goal:
					env.Remove(goalSample)
				push = PSM.MoveGripperStraight(distance= success[1], Tee = robottrans, dirAngle= v)
				for goalSample in goal:
					env.AddKinBody(goalSample)
				if push is not None:
					print "Push Grasp Found"
					data_file.write("\n%f" %(time.time()-start))
					start = time.time()
					time.sleep(3)
					break
			if push is not None:
				continue
		if push is not None:
			continue
	data_file.close()
	#IPython.embed() 


	#	GetPose 									+
	#	Check Capture Region  						+
	#		Check Push distance 					

	#	Find plan 									-

