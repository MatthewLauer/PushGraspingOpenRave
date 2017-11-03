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
        body.InitFromBoxes(numpy.array([[0,0,0,5.1,0.2,0.3]]), True)
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
    import IPython
    IPython.embed()

