#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov (rosen.diankov@gmail.com)
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Tests moving the end effctor of the manipulator in straight paths.

.. examplepre-block:: movehandstraight

Description
-----------

Shows how to use the MoveHandStraight basemanipulation command. The example picks a random trajectory of the end effector and tests if this trajectory is feasible to achieve in the robot.

.. examplepost-block:: movehandstraight
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import time
from itertools import izip
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
class handMover:

    def __init__(self):
        print "New handMover"
        #nothing to do here

    def movehandstraight(self, env, direction, Tee, stepsize=0.01):
        "Main example code."
        import sys
        print "*******************************************************"
        print sys.path
        print "*******************************************************"
        import IPython
        #IPython.embed()
        #env.Load("/data/wamtest1.env.xml")
        steps = 10.0 #@TODO

        robot = env.GetRobots()[0]
        with env:
            ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
            if not ikmodel.load():
                ikmodel.autogenerate()
            basemanip = interfaces.BaseManipulation(robot)
            taskmanip = interfaces.TaskManipulation(robot)
            robot.SetJointValues([0.0,0.0,0.0,0.0],ikmodel.manip.GetGripperIndices())
            Tstart = array([[ -1,  0,  0,   2.00000000e-01], [  0,0,   1, 6.30000000e-01], [  0,   1  , 0,   5.50000000e-02], [  0,0,0,1]])
            sol = ikmodel.manip.FindIKSolution(Tstart,IkFilterOptions.CheckEnvCollisions)
            robot.SetDOFValues(sol,ikmodel.manip.GetArmIndices())

        h = env.drawlinelist(array([Tee[0:3,3],Tee[0:3,3]+direction*steps*stepsize]),1)
        try:
            #IPython.embed()
            success = basemanip.MoveHandStraight(direction=direction,starteematrix=Tee,stepsize=stepsize,minsteps=steps-1,maxsteps=steps)
            params = (direction,Tee)
            h = env.drawlinelist(array([Tee[0:3,3],Tee[0:3,3]+direction*steps*stepsize]),4,[0,0,1])
            robot.WaitForController(0)
            return True
        except planning_error,e:
            return False