import openravepy
from lineExample import handMover
import numpy

if __name__ == "__main__":
    env = openravepy.Environment()
    env.Load('env.xml')
    env.SetViewer('qtcoin')
    viewer = env.GetViewer()
    viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
    robot = env.GetRobots()[0]
    dofs = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
    robot.SetActiveDOFValues(dofs)
    direction = numpy.asarray([.707, .707, 0])
    Tee = robot.GetManipulator('arm').GetTransform()
    lex = handMover()
    lex.movehandstraight(env, direction, Tee)
    import IPython
    IPython.embed()

