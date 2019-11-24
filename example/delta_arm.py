import gym
import gym_gazebo2
import time
import pdb
import random
import gym
import numpy as np
from collections import deque
from keras.models import Sequential,Model
from keras.models import load_model
from keras.layers import Dense,Input
from keras.optimizers import Adam
from ctypes import *


#Function Definition#
def pickAndPlace(targetPosition, bowlPosition):
    #```` Moving Towards Object```````#
    x0 = c_float(targetPosition[0]*1000)
    y0 = c_float(targetPosition[1]*1000)
    z0 = c_float((targetPosition[2]*1000) + 100)

    res = libDeltaKinematics.delta_calcInverse(x0,y0,z0)
    tgtPos = [res.theta1,res.theta3,res.theta2]
    tgtPos = np.array(tgtPos,dtype=np.float32)
    tgtPos = np.reshape(tgtPos,(3,))
    #print ('#1 GO to target position')
    observation, done = env.step(tgtPos)
    #`````Reached on the Top of the object``````#

    #```` Going Downwards to Pick Object```````#
    x0 = c_float(targetPosition[0]*1000)
    y0 = c_float(targetPosition[1]*1000)
    z0 = c_float((targetPosition[2]*1000))

    #Loading inverse kinematic library for delta arm
    res = libDeltaKinematics.delta_calcInverse(x0,y0,z0)
    tgtPos = [res.theta1,res.theta3,res.theta2]
    tgtPos = np.array(tgtPos,dtype=np.float32)
    tgtPos = np.reshape(tgtPos,(3,))

    #print ('#2 GO down')
    observation, done = env.step(tgtPos)
    #`````Reached at Target``````#

    #``````Pick up commands for Gripper`````#
    vel    = 2.0
    pos    = 0.775
    effort = 50.0
    #print ('#3 GO Grab')
    done = env.moveGripper(vel,pos,effort)
    #``````Target Picked`````#

    #```` Going Upwards with Picked Object```````#
    x0 = c_float(targetPosition[0]*1000)
    y0 = c_float(targetPosition[1]*1000)
    z0 = c_float((targetPosition[2]*1000) + 150)

    res = libDeltaKinematics.delta_calcInverse(x0,y0,z0)
    tgtPos = [res.theta1,res.theta3,res.theta2]
    tgtPos = np.array(tgtPos,dtype=np.float32)
    tgtPos = np.reshape(tgtPos,(3,))

    #print ('#4 GO up')
    observation, done = env.step(tgtPos)
    #``````In UP position with picked object````#


    #``````Moving Towards bowl location`````#
    #Bowl Location
    x0 = c_float(bowlPosition[0]*1000)
    y0 = c_float(bowlPosition[1]*1000)
    z0 = c_float(bowlPosition[2]*1000)

    res = libDeltaKinematics.delta_calcInverse(x0,y0,z0)
    tgtPos = [res.theta1,res.theta3,res.theta2]
    tgtPos = np.array(tgtPos,dtype=np.float32)
    tgtPos = np.reshape(tgtPos,(3,))

    #print ('#5 GO to bowl')
    observation, done = env.step(tgtPos)
    #``````Reached on the Top of the Bowl`````#

    #````Opening Gripper and Placing object in Bowl````#
    vel    = 50.0
    pos    = 0.0
    effort = 25.0

    #print ('#6 Drop object')
    done = env.moveGripper(vel,pos,effort)
    #````Object dropped in bowl````#



#************Main Function***************#

env = gym.make('MARA-v0')

#Wait For loading environemnt
pdb.set_trace()
libDeltaKinematics = CDLL("./ros2_mara_ws/install/lib/libdeltaKinematics.so") #Loading Delta Kinematics C library
class ReVal(Structure):
    _fields_ = [("theta1", c_float),("theta2", c_float),("theta3", c_float)]

libDeltaKinematics.delta_calcInverse.argtypes = [c_float, c_float, c_float]
libDeltaKinematics.delta_calcInverse.restype = ReVal

#~~~~~Enter Target-Object Position in (x,y,z)  and ~~~~~~~~~#
targetPosition1 = np.asarray([1.38081283544e-05, 0.209778773089, -0.5])  # Sharp_box 1 position
targetPosition2 = np.asarray([-0.0999984355292, 0.099996952657, -0.5])  # Sharp_box 2 position
targetPosition3 = np.asarray([0.0999976168609, 0.150001818126, -0.5])  # Sharp_box 3 position
targetPosition4 = np.asarray([0.0899073798131, 9.62382843279e-05, -0.5])  # Sharp_box 4 position
targetPosition5 = np.asarray([-0.00937140177285, 0.0580800116273, -0.5])  # Sharp_box 5 position

targetPostionList = [targetPosition1, targetPosition2, targetPosition3, targetPosition4, targetPosition5]

#~~~~~Enter Target-Place Position ~~~~~~~~~#
bowlPosition = np.asarray([0.28, -0.07, -0.4])

while (True):
    cnt = 1
    pdb.set_trace()
    for targets in targetPostionList:
        pickAndPlace(targets, bowlPosition)
        print ('Target Done', cnt)
        cnt+=1


