from env import *
from midlevel import DiscreteSystem

import numpy as np
from polytope import Polytope
import matplotlib.pyplot as plt
import matlab.engine
import pickle

n = 4
m = 5
T_mid = 0.5

cellDict = {}

# xmin = -1.3
# xmax = 0.7
# ymin = 1.5
# ymax = -1.5
env_xmin = -1.3
env_xmax = 0.7
env_ymin = -2.1
env_ymax = 0.9
delta = 0.03

A = np.block([[np.eye(2), T_mid * np.eye(2)], [np.zeros((2, 2)), np.eye(2)]])
B = np.block([[np.zeros((2, 2))], [np.eye(2)]])
C = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
sys = DiscreteSystem(A, B, C)

vmax = np.array([40, 40])/100
vmin = np.array([-40, -40])/100
dvmax = np.array([5, 5])/100
dvmin = np.array([-5, -5])/100
Hv = np.vstack((np.eye(2), -np.eye(2)))
hv = np.concatenate([vmax, -vmin])
Hv = np.array([
    [1, 1],
    [1, -1],
    [-1, 1],
    [-1, -1]
])
hv = np.array([40, 40, 40, 40])/100
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

fig, ax = plt.subplots(figsize=(10,6))
for key in cellDict:
    cellDict[key].positionConstraints.plot(ax, color=[0.8, 0.8, 0.8], alpha=0.3, linewidth=1.5, edgecolor='k')
    cellDict[key].controlInvariantPosSet.plot(ax, color=[0.6, 0.6, 0.6], alpha=0.3, linewidth=1.5, edgecolor='k')
ax.set_xlabel("X coordinate")
ax.set_ylabel("Y coordinate")
ax.set_title("Environment")
plt.grid(True)
plt.axis('auto')
plt.savefig("env.png")
plt.show()

with open('cellDict.pkl', 'wb') as f:
    pickle.dump(cellDict, f)

