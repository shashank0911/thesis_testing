from midlevel import *
from toplevel import *
from robots import *
from env import *

import numpy as np
from polytope import Polytope
import time
import pickle
import pandas as pd

T_mid = 0.5
low_steps = 10
t_vals = Time(T_mid, low_steps)

nx = 4
nu = 2
ny = 2
N = 15
simsteps = 60

A = np.block([[np.eye(2), T_mid * np.eye(2)], [np.zeros((2, 2)), np.eye(2)]])
B = np.block([[np.zeros((2, 2))], [np.eye(2)]])
C = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
sys = DiscreteSystem(A, B, C)
dim = Dimensions(nx, nu, ny, N, simsteps)

Q = np.diag([10, 10, 1, 1])
R = np.diag([1, 1])
Qf = Q

weight = Weight(Q, R, Qf)

#Kp=52, Kd=1.1, sigma=36: e=[-0.0007, -0.006]
Kp = 52 * np.eye(2)
Kd = 1.1 * np.eye(2)
sigma = 36

vmax = np.array([30, 30])
vmin = np.array([-30, -30])
dvmax = np.array([5, 5])
dvmin = np.array([-5, -5])
Hv = np.vstack((np.eye(2), -np.eye(2)))
hv = np.concatenate([vmax, -vmin])
Hdv = np.vstack((np.eye(2), -np.eye(2)))
hdv = np.concatenate([dvmax, -dvmin])

velocityConstraints = Polytope(Hv, hv)
inputConstraints = Polytope(Hdv, hdv)

with open('cellDict.pkl', 'rb') as f:
    cellDict = pickle.load(f)

# idList = ["1234", "2468", "1357"]
# x01List = np.array([5, 5, 0, 0])
# x02List = np.array([10, 85, 0, 0])
# x03List = np.array([150, 66, 0, 0])
# x0List = np.vstack((x01List, x02List, x03List))

idList = ["1234"]
x0List = np.array([5, 5, 0, 0]).reshape(1,-1)

robots = Robots(idList, x0List, t_vals, cellDict)
robots.initialise_mid_level_system(dim)
robots.initialise_low_level_system()

# motionPlan1 = ["c_41", "c_42", "c_32", "c_31", "c_21"]
# motionPlan2 = ["c_11", "c_12", "c_22", "c_32"]
# motionPlan3 = ["c_24", "c_25", "c_35", "c_45", "c_44", "c_34", "c_24"]

motionPlan1 = ["c_41", "c_42"]
# motionPlan2 = ["c_11", "c_12"]
# motionPlan3 = ["c_24", "c_25"]

robots.robotList[0].topSystem = TopLevelSystem(motionPlan1)
# robots.robotList[1].topSystem = TopLevelSystem(motionPlan2)
# robots.robotList[2].topSystem = TopLevelSystem(motionPlan3)

robots.initialise_mid_level_control(weight, velocityConstraints, inputConstraints)
robots.initialise_low_level_control(Kp, Kd, sigma)

execTimeMid = np.zeros(simsteps)

t = 0
mid_layer_loop_time = t_vals.T_mid
low_layer_loop_time = t_vals.T_low
mid_layer_time = time.perf_counter()
low_layer_time = time.perf_counter()
break_condition = False


# Initialization 
# start_time = time.perf_counter()
# for i in range(robots.numRobots):

#     if robots.robotList[i].midControl.terminate:
#         sequencePair = robots.robotList[i].topSystem.get_sequence_pair()
#         if robots.robotList[i].topSystem.terminate:
#             if robots.robotList[i].midSystem.t < simsteps:
#                 xc = np.concatenate([robots.robotList[i].midSystem.xc.flatten()[:2], np.zeros(2)])
#                 # print(f"Loop number: {robots.robotList[i].midSystem.t+1}, Robot:{i+1}")
#                 robots.robotList[i].midSystem.stateHistory[:, robots.robotList[i].midSystem.t] = xc
#                 robots.robotList[i].midSystem.inputHistory[:, robots.robotList[i].midSystem.t] = np.zeros(2)
#                 robots.robotList[i].midSystem.t += 1
#             continue
#         else:
#             highLevelInput = robots.find_high_level_input(sequencePair)
#             robots.robotList[i].midControl.get_constraints(highLevelInput)
#             robots.robotList[i].midSystem.xf = highLevelInput[1].center
#             robots.robotList[i].midControl.terminate = False

# print(f"Loop number: {robots.robotList[i].midSystem.t+1}, Robot:{i+1}")
# for i in range(robots.numRobots):
#     # robots.robotList[i].lowSystem.k += 1
#     if robots.robotList[i].midSystem.t < simsteps:
#         robots.robotList[i].midSystem.get_mpc(robots.robotList[i].midControl)
#         robots.robotList[i].midSystem.t += 1
# elapsedTime = time.perf_counter() - start_time
# execTimeMid[t] = elapsedTime
# t += 1
# # if robots.robotList[0].lowSystem.k == 0:
# low_layer_start = time.perf_counter()
# # Initialization end

while not break_condition:
    
    start_time = time.perf_counter()

    for i in range(robots.numRobots):

        if robots.robotList[i].midControl.terminate:
            sequencePair = robots.robotList[i].topSystem.get_sequence_pair()
            if robots.robotList[i].topSystem.terminate:
                if robots.robotList[i].midSystem.t < simsteps:
                    xc = np.concatenate([robots.robotList[i].midSystem.xc.flatten()[:2], np.zeros(2)])
                    # print(f"Loop number: {robots.robotList[i].midSystem.t+1}, Robot:{i+1}")
                    robots.robotList[i].midSystem.stateHistory[:, robots.robotList[i].midSystem.t] = xc
                    robots.robotList[i].midSystem.inputHistory[:, robots.robotList[i].midSystem.t] = np.zeros(2)
                    robots.robotList[i].midSystem.t += 1
                continue
            else:
                highLevelInput = robots.find_high_level_input(sequencePair)
                robots.robotList[i].midControl.get_constraints(highLevelInput)
                robots.robotList[i].midSystem.xf = highLevelInput[1].center
                robots.robotList[i].midControl.terminate = False
        
    # Middle layer loop     
       
    if start_time - mid_layer_time >= mid_layer_loop_time:
        mid_layer_time = time.perf_counter()
        # if robots.robotList[0].lowSystem.k == -1:
        #     low_layer_start = time.perf_counter()
        for i in range(robots.numRobots):
            print(f"Loop number: {robots.robotList[i].midSystem.t+1}, Robot:{i+1}")
            robots.robotList[i].lowSystem.k += 1
            if robots.robotList[i].midSystem.t < simsteps:
                robots.robotList[i].midSystem.get_mpc(robots.robotList[i].midControl)
                robots.robotList[i].midSystem.t += 1
        elapsedTime = time.perf_counter() - start_time
        execTimeMid[t] = elapsedTime
        t += 1
        if robots.robotList[0].lowSystem.k == 0:
            low_layer_start = time.perf_counter()
        # print(break_condition)

    # Bottom layer loop
    low_buffer = 0.002 
    if (robots.robotList[0].lowSystem.k != -1) and (start_time - low_layer_time >= low_layer_loop_time):
        low_layer_time = time.perf_counter()
        
        for i in range(robots.numRobots):
            # if (robots.robotList[i].lowSystem.t + low_layer_loop_time) < (robots.robotList[0].lowSystem.k + 1) * (mid_layer_loop_time - low_buffer):
            robots.robotList[i].lowSystem.t = time.perf_counter() - low_layer_start
            robots.robotList[i].lowSystem.k = int(robots.robotList[i].lowSystem.t / mid_layer_loop_time)
            # robots.robotList[i].lowSystem.t += low_layer_loop_time
            if robots.robotList[i].midSystem.t < simsteps:
                robots.robotList[i].lowSystem.update_state(robots.robotList[i].lowControl, robots.robotList[i].midSystem.yc)
            else:
                robots.robotList[i].lowSystem.stateHistory.append(robots.robotList[i].lowSystem.stateHistory[-1])
                robots.robotList[i].lowSystem.inputHistory.append(robots.robotList[i].lowSystem.inputHistory[-1])
                robots.robotList[i].lowSystem.trajectoryHistory.append(robots.robotList[i].lowSystem.trajectoryHistory[-1])
            

    break_condition = all(robots.robotList[i].midSystem.t > simsteps-1 for i in range(robots.numRobots))



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
    stateHistoryLow = np.squeeze(stateHistoryLow).T
    inputHistoryLow = np.squeeze(inputHistoryLow).T
    trajectoryHistoryLow = np.squeeze(trajectoryHistoryLow).T
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

df = pd.DataFrame(execTimeMid, columns=["Execution Time"])
df.to_csv("execTimeMid.csv", index=False)
