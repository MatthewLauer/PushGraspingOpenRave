from openravepy import *
import openravepy
from lineExample import handMover
import numpy
from PushStateMachine import PushStateMachine

if __name__ == "__main__":
    env = openravepy.Environment()
    env.Load('env.xml')
    env.SetViewer('qtcoin')

    with env:
        body = RaveCreateKinBody(env,'')
        body.SetName("box");
        body.InitFromBoxes(numpy.array([[0,0,0,.1,0.05,0.075]]), True)
        body.SetTransform(numpy.array([[1,0,0,-.75],[0,1,0,1.0],[0,0,1,1.2], [0,0,0,1]]))
        env.AddKinBody(body)

    viewer = env.GetViewer()
    viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
    robot = env.GetRobots()[0]
    dofs = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
    robot.SetActiveDOFValues(dofs)
    direction = numpy.asarray([.707, .707, 0])
    Tee = robot.GetManipulator('arm').GetTransform()
    lex = handMover()
    lex.movehandstraight(env, direction, Tee)

    PSM = PushStateMachine(env)
    PSM.GetPoses(env.GetKinBody('box'),1,1,1)
    import IPython
    IPython.embed()

