import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
from lineExample import handMover
import numpy
import IPython
import time
import math

class PushStateMachine:

    def __init__(self, env):
        self.handMover = handMover();
        self.env = env;
        self.robot = env.GetRobots()[0]
        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=self.robot,iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()
            self.basemanip = interfaces.BaseManipulation(self.robot)
            self.robot.SetJointValues([0.0,0.0,0.0,0.0],self.ikmodel.manip.GetGripperIndices())
            Tstart = array([[ -1,  0,  0,   2.00000000e-01], [  0,0,   1, 6.30000000e-01], [  0,   1  , 0,   5.50000000e-02], [  0,0,0,1]])
            sol = self.ikmodel.manip.FindIKSolution(Tstart,IkFilterOptions.CheckEnvCollisions)
            IPython.embed()
            self.robot.SetDOFValues(sol,self.ikmodel.manip.GetArmIndices())
  

    #Takes an object in the scene and returns an array of tuples 
    # with suggested hand pose (x, y, theta)
    # kinboayd is the object you want 
    def GetPoses(self, kinbody, minAperture, vStepCount=20,  oStepCount=1, backwardincrement = .01):
        print 'getting params'#does nothign
        trans = kinbody.GetTransform()
        sol = numpy.array([0,0,0,0,0,0,0])#7Dof
        oSteps = numpy.linspace(-(self._ApetureAngleToDistance(0)-self._ApetureAngleToDistance(minAperture)),
        	(self._ApetureAngleToDistance(0)-self._ApetureAngleToDistance(minAperture)), num=oStepCount*2+1)
        vSteps = numpy.linspace(0,2*math.pi, num = vStepCount, endpoint = False)
        aSteps = numpy.linspace(0,minAperture,num = round(abs(0-minAperture)/.2));
        armPoses = numpy.empty([0,11])

        #IPython.embed()
        #aSteps = [2]
        #oSteps = [0]
        #vSteps = [0]
        for o in oSteps:
        	for v in vSteps:
        		for a in aSteps:
        			dofs = self.robot.GetActiveDOFValues()
        			dofs[7] = a;
        			dofs[8] = a;
        			dofs[9] = a;
        			self.robot.SetActiveDOFValues(dofs)
        			rotatedtrans = numpy.array(trans)
        			rotatedtrans[0:3,0:3] = numpy.dot(matrixFromAxisAngle([0,0,v])[0:3,0:3], trans[0:3,0:3])
        			rotatedtrans[1][3] = rotatedtrans[1][3] + math.sin(v)*o        			
        			rotatedtrans[0][3] = rotatedtrans[0][3] + math.cos(v)*o        			
					
        			print 'o: %f v: %f a: %f' % (o,v,a)
        			sol = self.ikmodel.manip.FindIKSolution(rotatedtrans, IkFilterOptions.IgnoreEndEffectorCollisions)
        			if sol is None:
        				print 'fail'
        				break;
        			#IPython.embed()
        			while True:
        				rotatedtrans[1][3] = rotatedtrans[1][3] + math.cos(v)*backwardincrement      			
        				rotatedtrans[0][3] = rotatedtrans[0][3] - math.sin(v)*backwardincrement        			
        				#IPython.embed()
        				anysol = self.ikmodel.manip.FindIKSolution(rotatedtrans, IkFilterOptions.IgnoreEndEffectorCollisions)
        				if anysol is None:
        					print 'fail'
        					break
        				if anysol is not None:
        					sol11DOF = numpy.append(anysol,self.robot.GetActiveDOFValues()[7:11])				
        					self.robot.SetActiveDOFValues(sol11DOF)
        					time.sleep(.1)
        				colsol = self.ikmodel.manip.FindIKSolution(rotatedtrans, IkFilterOptions.CheckEnvCollisions)
        				if colsol is not None:
        					sol11DOF = numpy.append(colsol,self.robot.GetActiveDOFValues()[7:11])				
        					armPoses = numpy.vstack([armPoses, sol11DOF])
        					self.robot.SetActiveDOFValues(sol11DOF)
        					time.sleep(.1)
        					print 'success'
        					break

       	#while True:
        #    sol = self.ikmodel.manip.FindIKSolution(trans, IkFilterOptions.IgnoreEndEffectorCollisions)
    	#    sol11DOF = numpy.append(sol,self.robot.GetActiveDOFValues()[7:11])
        #    self.robot.SetActiveDOFValues(sol11DOF)
        #    IPython.embed()
        IPython.embed()
        return armPoses

    def sampleTransformXY(self, transform, sigma = .02):
    	trans = transform.copy()
    	trans[1][3] = numpy.random.normal(trans[1][3], sigma, 1)
    	trans[0][3] = numpy.random.normal(trans[0][3], sigma, 1)
    	return trans


    #Terrible approxiamation that is sufficient for now
    def _ApetureAngleToDistance(self,angle):
    	return 2*.075*math.cos(angle)


    def MoveGripper(self, transform):
    	try:
    		print 'attempting to move hand'
    		#success = self.basemanip.MoveToHandPosition(translation=None,rotation=None,execute=True,outputtraj=True,minimumgoalpaths=1):
        except planning_error,e:
            return None



if __name__ == "__main__":
	env = openravepy.Environment()
	env.Load('env.xml')
	env.SetViewer('qtcoin')
	PSM = PushStateMachine(env)

	body = []
	with env:
		boxtrans = numpy.array([[1,0,0,-.6785],[0,0,-1,-.6],[0,1,.0,1.112], [0,0,0,1]])
		for i in range(10):
			tempbody = RaveCreateKinBody(env,'')
			tempbody.SetName(("Box" + str(i)))
			tempbody.InitFromBoxes(numpy.array([[0,0,0,.1,0.05,0.075]]), True)
			trans = PSM.sampleTransformXY(boxtrans)
			tempbody.SetTransform(trans)
			body.append(tempbody)
			env.AddKinBody(body[i])


	with env:
		body = RaveCreateKinBody(env,'')
		body.SetName("box");
		body.InitFromBoxes(numpy.array([[0,0,0,.1,0.05,0.075]]), True)
		#body.SetTransform(numpy.array([[1,0,0,-.75],[0,1,0,1.0],[0,0,1,1.2], [0,0,0,1]]))
		#body.SetTransform(numpy.array([[.9846,.1600,.0706,-.6785],[.0581,.0823,-.9950,-.6],
		#    [-.1649,.9838,.0707,1.112], [0,0,0,1]]))
		body.SetTransform(numpy.array([[1,0,0,-.6785],[0,0,-1,-.3],[0,1,.0,1.112], [0,0,0,1]]))
		env.AddKinBody(body)

	viewer = env.GetViewer()
	viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
	robot = env.GetRobots()[0]
	dofs = [2.0,-1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
	robot.SetActiveDOFValues(dofs)

#More Initialization							-

    #sample goal object 							+

    #sample obstacles								

    #Loop through offset parameters a,v,o 			-
    #	GetPose 									+
    #	Check Capture Region  						-
    #		Check Push distance 					

	#	Find plan 									-


	import IPython
	IPython.embed()