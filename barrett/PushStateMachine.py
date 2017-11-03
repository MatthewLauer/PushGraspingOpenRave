import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
from lineExample import handMover
import numpy
import IPython

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
            self.robot.SetDOFValues(sol,self.ikmodel.manip.GetArmIndices())
  

    #Takes an object in the scene and returns an array of tuples 
    # with suggested hand pose (x, y, theta)
    # kinboayd is the object you want 
    def GetParamS(self, kinbody, v, a, o):
       print 'getting params'#does nothign


    def MoveGripper(self, transform):
    	try:
    		print 'attempting to move hand'#success = basemanip.MoveToHandPosition(translation=None,rotation=None,,execute=True,outputtraj=True,minimumgoalpaths=1):
        except planning_error,e:
            return None



if __name__ == "__main__":
    env = openravepy.Environment()
    env.Load('env.xml')
    env.SetViewer('qtcoin')
    PSM = PushStateMachine(env)