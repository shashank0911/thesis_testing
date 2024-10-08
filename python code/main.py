from midlevel import *
from toplevel import *
from robots import *
from env import *

import numpy as np
from polytope import Polytope
import time
import pickle
import pandas as pd

T_mid = 1
low_steps = 20
simsteps = 60
t_vals = Time(T_mid, low_steps, simsteps)

nx = 4
nu = 2
ny = 2
N = 15


A = np.block([[np.eye(2), T_mid * np.eye(2)], [np.zeros((2, 2)), np.eye(2)]])
B = np.block([[np.zeros((2, 2))], [np.eye(2)]])
C = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
sys = DiscreteSystem(A, B, C)
dim = Dimensions(nx, nu, ny, N, simsteps)

Q = np.diag([10, 10, 1, 1])
R = np.diag([1, 1])
Qf = Q

weight = Weight(Q, R, Qf)

# Kp=52, Kd=1.1, sigma=36: e=[-0.0007, -0.006] for c_41 to c_42 - useless now
# Kp=25, Kd=1.7, sigma=25 for Tmid=0.5, lowsteps=10, c_41, c_31
# Kp=25, Kd=1.7, sigma=25 for Tmid=0.5, lowsteps=10, v=0.01, c_41, c_42, (-1.2,-0.7), (-0.9, -0.7)
Kp = 8 * np.eye(2)
Kd = 1.64 * np.eye(2)
sigma = 17
# kp=50, s=40, kd=2.2, Tmid=0.5, lowsteps=10, v=0.01, c_41, c_31, (-1.2, -0.7), (-0.9, -0.7)

vmax = np.array([30, 30])/100
vmin = np.array([-30, -30])/100
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

with open('cellDict.pkl', 'rb') as f:
    cellDict = pickle.load(f)

idList = ["1234", "2468", "1357"]
x01List = np.array([-1.2, -1.4, 0, 0])
x02List = np.array([-1.15, 0.8, 0, 0])
x03List = np.array([0.6, 0.1, 0, 0])
x0List = np.vstack((x01List, x02List, x03List))

# idList = ["1234"]
# x0List = np.array([-1.2, -1.4, 0, 0]).reshape(1,-1)

robots = Robots(idList, x0List, t_vals, cellDict)
robots.initialise_mid_level_system(dim)
robots.initialise_low_level_system()

# motionPlan1 = ["c_41", "c_42", "c_32", "c_31", "c_21"]
# motionPlan2 = ["c_11", "c_12", "c_22", "c_32"]
# motionPlan3 = ["c_25", "c_35", "c_45", "c_44", "c_34", "c_24"]

motionPlan1 = ["c_41", "c_31"]
motionPlan2 = ["c_11", "c_21"]
motionPlan3 = ["c_24", "c_23", "c_33"]

robots.robotList[0].topSystem = TopLevelSystem(motionPlan1)
robots.robotList[1].topSystem = TopLevelSystem(motionPlan2)
robots.robotList[2].topSystem = TopLevelSystem(motionPlan3)

robots.initialise_mid_level_control(weight, velocityConstraints, inputConstraints)
robots.initialise_low_level_control(Kp, Kd, sigma)

execTimeMid = np.zeros(simsteps)
execTimeLow = np.zeros(simsteps*low_steps)

t = 0
mid_layer_loop_time = t_vals.T_mid
low_layer_loop_time = t_vals.T_low
mid_layer_time = time.perf_counter()
low_layer_time = time.perf_counter()
# break_condition = False

# while not break_condition:
    
#     start_time = time.perf_counter()

#     for i in range(robots.numRobots):

#         if robots.robotList[i].midControl.terminate:
#             sequencePair = robots.robotList[i].topSystem.get_sequence_pair()
#             if robots.robotList[i].topSystem.terminate:
#                 continue
#             else:
#                 highLevelInput = robots.find_high_level_input(sequencePair)
#                 robots.robotList[i].midControl.get_constraints(highLevelInput)
#                 robots.robotList[i].midSystem.xf = highLevelInput[1].center
#                 robots.robotList[i].midControl.terminate = False
        
#     # Middle layer loop     
       
#     if start_time - mid_layer_time >= mid_layer_loop_time:
#         mid_layer_time = time.perf_counter()
#         # if robots.robotList[0].lowSystem.k == -1:
#         #     low_layer_start = time.perf_counter()
#         for i in range(robots.numRobots):
#             print(f"Loop number: {robots.robotList[i].midSystem.t+1}, Robot:{i+1}")
#             robots.robotList[i].lowSystem.k += 1
#             if robots.robotList[i].midSystem.t < simsteps:
#                 robots.robotList[i].midSystem.get_mpc(robots.robotList[i].midControl)
#                 robots.robotList[i].midSystem.t += 1
#         elapsedTimeMid = time.perf_counter() - start_time
#         execTimeMid[t] = elapsedTimeMid
#         t += 1
#         if robots.robotList[0].lowSystem.k == 0:
#             low_layer_start = time.perf_counter()
#         # print(break_condition)

#     # Bottom layer loop
#     low_buffer = 0.002 
#     if (robots.robotList[0].lowSystem.k != -1) and (start_time - low_layer_time >= low_layer_loop_time):
#         low_layer_time = time.perf_counter()
        
#         for i in range(robots.numRobots):
#             # if (robots.robotList[i].lowSystem.t + low_layer_loop_time) < (robots.robotList[0].lowSystem.k + 1) * (mid_layer_loop_time - low_buffer):
#             robots.robotList[i].lowSystem.t = time.perf_counter() - low_layer_start
#             robots.robotList[i].lowSystem.k = int(robots.robotList[i].lowSystem.t / mid_layer_loop_time)
#             # robots.robotList[i].lowSystem.t += low_layer_loop_time
#             if robots.robotList[i].midSystem.t < simsteps:
#                 robots.robotList[i].lowSystem.update_state(robots.robotList[i].lowControl, robots.robotList[i].midSystem.yc)
#                 robots.robotList[i].lowSystem.ct += 1
            
#         elapsedTimeLow = time.perf_counter() - low_layer_time
#         idx = robots.robotList[i].lowSystem.ct
#         execTimeLow[idx-1] = elapsedTimeLow


#     # break_condition = all(robots.robotList[i].midSystem.t > simsteps-1 for i in range(robots.numRobots))
#     break_condition = all(robots.robotList[i].topSystem.terminate for i in range(robots.numRobots))

for t in range(simsteps):
    mid_start_time = time.perf_counter()
    for i in range(robots.numRobots):

        if robots.robotList[i].midControl.terminate:
            sequencePair = robots.robotList[i].topSystem.get_sequence_pair()
            if robots.robotList[i].topSystem.terminate:
                continue
            else:
                highLevelInput = robots.find_high_level_input(sequencePair)
                robots.robotList[i].midControl.get_constraints(highLevelInput)
                robots.robotList[i].midSystem.xf = highLevelInput[1].center
                robots.robotList[i].midControl.terminate = False
        
        robots.robotList[i].midSystem.t = t
        robots.robotList[i].midSystem.get_mpc(robots.robotList[i].midControl)
        # for j in range(low_steps):
        #     robots.robotList[i].lowSystem.t = t_vals.T_low_arr[robots.robotList[i].lowSystem.ct]
        #     robots.robotList[i].lowSystem.k = int(robots.robotList[i].lowSystem.t / t_vals.T_mid)
        #     robots.robotList[i].lowSystem.update_state(robots.robotList[i].lowControl, robots.robotList[i].midSystem.yc)
        #     robots.robotList[i].lowSystem.ct += 1

    execTimeMid[t] = time.perf_counter() - mid_start_time

    
    for j in range(low_steps):
        low_start_time = time.perf_counter()
        for i in range(robots.numRobots):
            if robots.robotList[i].topSystem.terminate:
                continue
            else:
                robots.robotList[i].lowSystem.t = t_vals.T_low_arr[robots.robotList[i].lowSystem.ct]
                robots.robotList[i].lowSystem.k = int(robots.robotList[i].lowSystem.t / t_vals.T_mid)
                robots.robotList[i].lowSystem.update_state(robots.robotList[i].lowControl, robots.robotList[i].midSystem.yc)
                robots.robotList[i].lowSystem.ct += 1
        execTimeLow[robots.robotList[i].lowSystem.ct] = time.perf_counter() - low_start_time

    if all(robots.robotList[k].topSystem.terminate for k in range(robots.numRobots)):
        break

for i in range(robots.numRobots):

    if robots.robotList[i].midSystem.t < simsteps:
        xc = np.concatenate([robots.robotList[i].midSystem.xc.flatten()[:2], np.zeros(2)])
        # print(f"Loop number: {robots.robotList[i].midSystem.t+1}, Robot:{i+1}")
        robots.robotList[i].midSystem.stateHistory[:, robots.robotList[i].midSystem.t:] = xc.reshape(-1, 1)
        robots.robotList[i].midSystem.inputHistory[:, robots.robotList[i].midSystem.t:] = np.zeros((2, 1))

    ct = robots.robotList[i].lowSystem.ct
    xc_low = np.concatenate([robots.robotList[i].midSystem.xc.flatten()[:3], np.zeros(2)])
    robots.robotList[i].lowSystem.stateHistory[:, ct+1:] = xc_low.reshape(-1, 1)
    robots.robotList[i].lowSystem.inputHistory[:, ct:] = np.zeros((2, 1))
    robots.robotList[i].lowSystem.trajectoryHistory[:, ct:] = robots.robotList[i].lowSystem.trajectoryHistory[:, ct-1].reshape(-1, 1)


robotData = {}
for i in range(robots.numRobots):
    stateHistoryMid = robots.robotList[i].midSystem.stateHistory
    inputHistoryMid = robots.robotList[i].midSystem.inputHistory
    stateHistoryLow = robots.robotList[i].lowSystem.stateHistory
    inputHistoryLow = robots.robotList[i].lowSystem.inputHistory
    trajectoryHistoryLow = robots.robotList[i].lowSystem.trajectoryHistory
    # print(np.shape(stateHistoryLow))
    # print(np.shape(inputHistoryLow))
    # print(np.shape(trajectoryHistoryLow))
    # print(np.shape(stateHistoryMid))
    # print(np.shape(inputHistoryMid))
    # stateHistoryLow = np.squeeze(stateHistoryLow).T
    # inputHistoryLow = np.squeeze(inputHistoryLow).T
    # trajectoryHistoryLow = np.squeeze(trajectoryHistoryLow).T
    robotData[f'stateHistoryMid_{i}'] = stateHistoryMid
    robotData[f'inputHistoryMid_{i}'] = inputHistoryMid
    robotData[f'stateHistoryLow_{i}'] = stateHistoryLow
    robotData[f'inputHistoryLow_{i}'] = inputHistoryLow
    robotData[f'trajectoryHistoryLow_{i}'] = trajectoryHistoryLow
    
robotData[f'numRobots'] = robots.numRobots


cellData = {}
for key in cellDict:
    constSet_A = cellDict[key].positionConstraints.A
    constSet_b = cellDict[key].positionConstraints.b
    invSet_A = cellDict[key].controlInvariantPosSet.A
    invSet_b = cellDict[key].controlInvariantPosSet.b

    cellData[key] = {
        'constraintPos_A': constSet_A,
        'constraintPos_b': constSet_b,
        'invSet_A': invSet_A,
        'invSet_b': invSet_b
    }

with open('robot_cell_data.pkl', 'wb') as f:
    pickle.dump({'robot_data': robotData, 'cell_data': cellData}, f)

df1 = pd.DataFrame(execTimeMid, columns=["Execution Time"])
df1.to_csv("execTimeMid.csv", index=False)

df2 = pd.DataFrame(execTimeLow, columns=["Execution Time"])
df2.to_csv("execTimeLow.csv", index=False)