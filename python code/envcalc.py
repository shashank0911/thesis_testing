from env import *
from midlevel import DiscreteSystem

import numpy as np
from polytope import Polytope
import matlab.engine
import pickle

n = 5
m = 4
T_mid = 0.5

cellDict = {}

env_xmin = 0
env_xmax = 200
env_ymin = 0
env_ymax = 100
delta = 3

A = np.block([[np.eye(2), T_mid * np.eye(2)], [np.zeros((2, 2)), np.eye(2)]])
B = np.block([[np.zeros((2, 2))], [np.eye(2)]])
C = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
sys = DiscreteSystem(A, B, C)

vmax = np.array([40, 40])
vmin = np.array([-40, -40])
dvmax = np.array([5, 5])
dvmin = np.array([-5, -5])
Hv = np.vstack((np.eye(2), -np.eye(2)))
hv = np.concatenate([vmax, -vmin])
Hdv = np.vstack((np.eye(2), -np.eye(2)))
hdv = np.concatenate([dvmax, -dvmin])

velocityConstraints = Polytope(Hv, hv)
inputConstraints = Polytope(Hdv, hdv)

xbound = np.linspace(env_xmin, env_xmax, n + 1)
ybound = np.linspace(env_ymax, env_ymin, m + 1)

eng = matlab.engine.start_matlab()
eng.run('../matlab code/tbxmanager/startup.m', nargout=0)
for i in range(n):
    for j in range(m):
        xmin = xbound[i]
        xmax = xbound[i+1]
        ymax = ybound[j]
        ymin = ybound[j+1]
        keyString = f'c_{j+1}{i+1}'
        cellDict[keyString] = EnvCell(i+1, j+1, xmin, xmax, ymin, ymax, sys, 
                                      velocityConstraints, inputConstraints, delta, eng)
eng.quit()

with open('cellDict.pkl', 'wb') as f:
    pickle.dump(cellDict, f)

