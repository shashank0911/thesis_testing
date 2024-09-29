from midlevel import *
from toplevel import *
from robots import *
from env import *

import numpy as np
from polytope import Polytope
import time
# import matplotlib.pyplot as plt
# import matlab.engine
import pickle
import pandas as pd

# n = 5
# m = 4
T_mid = 0.5
low_steps = 10
t_vals = Time(T_mid, low_steps)

# cellDict = {}

# env_xmin = 0
# env_xmax = 200
# env_ymin = 0
# env_ymax = 100
# delta = 3

nx = 4
nu = 2
ny = 2
N = 20
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

Kp = 2 * np.eye(2)
Kd = np.eye(2)
sigma = 1

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

# xbound = np.linspace(env_xmin, env_xmax, n + 1)
# ybound = np.linspace(env_ymax, env_ymin, m + 1)

# eng = matlab.engine.start_matlab()
# eng.run('../matlab code/tbxmanager/startup.m', nargout=0)
# for i in range(n):
#     for j in range(m):
#         xmin = xbound[i]
#         xmax = xbound[i+1]
#         ymax = ybound[j]
#         ymin = ybound[j+1]
#         keyString = f'c_{j+1}{i+1}'
#         cellDict[keyString] = EnvCell(i+1, j+1, xmin, xmax, ymin, ymax, sys, 
#                                       velocityConstraints, inputConstraints, delta, eng)
# eng.quit()

with open('cellDict.pkl', 'rb') as f:
    cellDict = pickle.load(f)

idList = ["1234", "2468", "1357"]
x01List = np.array([5, 5, 0, 0])
x02List = np.array([10, 85, 0, 0])
x03List = np.array([150, 66, 0, 0])
x0List = np.vstack((x01List, x02List, x03List))

# idList = ["1234"]
# x0List = np.array([5, 5, 0, 0]).reshape(1,-1)

robots = Robots(idList, x0List, t_vals, cellDict)
robots.initialise_mid_level_system(dim)
robots.initialise_low_level_system()

motionPlan1 = ["c_41", "c_42", "c_43", "c_33", "c_34"]
motionPlan2 = ["c_11", "c_12", "c_22", "c_32"]
motionPlan3 = ["c_24", "c_25", "c_35", "c_45", "c_44", "c_34", "c_24"]

robots.robotList[0].topSystem = TopLevelSystem(motionPlan1)
robots.robotList[1].topSystem = TopLevelSystem(motionPlan2)
robots.robotList[2].topSystem = TopLevelSystem(motionPlan3)

robots.initialise_mid_level_control(weight, velocityConstraints, inputConstraints)
robots.initialise_low_level_control(Kp, Kd, sigma)

execTimeArr = np.zeros(simsteps)


# for i in range(robots.numRobots):
#     robots.robotList[i].mid_layer_loop_time = mid_layer_loop_time
#     robots.robotList[i].low_layer_loop_time = low_layer_loop_time
#     robots.robotList[i].mid_layer_time = time.perf_counter()
#     robots.robotList[i].low_layer_time = time.perf_counter()


# loop_terminate = np.full(robots.numRobots, False)
# while not np.all(loop_terminate):
#     start_time = time.perf_counter()
#     # loop_terminate = True
#     for i in range(robots.numRobots):
#         # robots.robotList[i].midSystem.t = t

#         if robots.robotList[i].midControl.terminate:
#             sequencePair = robots.robotList[i].topSystem.get_sequence_pair()
#             if robots.robotList[i].topSystem.terminate:
#                 xc = np.concatenate([robots.robotList[i].midSystem.xc.flatten()[:2], np.zeros(2)])
#                 robots.robotList[i].midSystem.stateHistory[:, t] = xc
#                 robots.robotList[i].midSystem.inputHistory[:, t] = np.zeros(2)
                
#                 if robots.robotList[i].midSystem.t > simsteps-1:
#                     loop_terminate[i] = True
#                     # break
#                 robots.robotList[i].midSystem.t += 1
#                 print(f"Loop number: {robots.robotList[i].midSystem.t}, Robot:{i+1}")
#                 # elapsedTime = time.perf_counter() - start_time
#                 # execTimeArr[t] = elapsedTime
#                 continue

#             else:
#                 highLevelInput = robots.find_high_level_input(sequencePair)
#                 robots.robotList[i].midControl.get_constraints(highLevelInput)
#                 robots.robotList[i].midSystem.xf = highLevelInput[1].center
#                 robots.robotList[i].midControl.terminate = False

    
#         if start_time - robots.robotList[i].mid_layer_time >= robots.robotList[i].mid_layer_loop_time:
#             robots.robotList[i].midSystem.t += 1
#             print(f"Loop number: {robots.robotList[i].midSystem.t}, Robot:{i+1}")
#             if robots.robotList[i].midSystem.t > simsteps-1:
#                 loop_terminate[i] = True
#                 continue
#             robots.robotList[i].mid_layer_time = time.perf_counter()
#             robots.robotList[i].midSystem.get_mpc(robots.robotList[i].midControl)
            
#             elapsedTime = time.perf_counter() - start_time
#             execTimeArr[t] = elapsedTime
            
        

#         # if start_time - low_layer_time >= low_layer_loop_time:
#         #     robots.robotList[i].lowSystem.update_state(robots.robotList[i].lowControl, robots.robotList[i].midSystem.yc)
#     time.sleep(0.01)


t = 0
mid_layer_loop_time = 0.5
low_layer_loop_time = 0.05
low_layer_time = time.perf_counter()
mid_layer_time = time.perf_counter()
break_condition = False

# for k in range(simsteps):
while not break_condition:
    
    start_time = time.perf_counter()

    for i in range(robots.numRobots):
        # robots.robotList[i].midSystem.t = t

        if robots.robotList[i].midControl.terminate:
            sequencePair = robots.robotList[i].topSystem.get_sequence_pair()
            if robots.robotList[i].topSystem.terminate:
                if robots.robotList[i].midSystem.t < simsteps:
                    xc = np.concatenate([robots.robotList[i].midSystem.xc.flatten()[:2], np.zeros(2)])
                    print(f"Loop number: {robots.robotList[i].midSystem.t+1}, Robot:{i+1}")
                    robots.robotList[i].midSystem.stateHistory[:, robots.robotList[i].midSystem.t] = xc
                    robots.robotList[i].midSystem.inputHistory[:, robots.robotList[i].midSystem.t] = np.zeros(2)
                    # print(f"Loop number: {robots.robotList[i].midSystem.t+1}, Robot:{i+1}, Xc: {robots.robotList[i].midSystem.stateHistory[:2, robots.robotList[i].midSystem.t]}")
                    robots.robotList[i].midSystem.t += 1
                continue
            else:
                highLevelInput = robots.find_high_level_input(sequencePair)
                robots.robotList[i].midControl.get_constraints(highLevelInput)
                robots.robotList[i].midSystem.xf = highLevelInput[1].center
                robots.robotList[i].midControl.terminate = False
        
        
    if start_time - mid_layer_time >= mid_layer_loop_time:
        mid_layer_time = time.perf_counter()
        print(f"Loop number: {robots.robotList[i].midSystem.t+1}, Robot:{i+1}")
        for i in range(robots.numRobots):
            if robots.robotList[i].midSystem.t < simsteps:
                robots.robotList[i].midSystem.get_mpc(robots.robotList[i].midControl)
                robots.robotList[i].midSystem.t += 1
        # robots.robotList[i].midSystem.t = t
        elapsedTime = time.perf_counter() - start_time
        execTimeArr[t] = elapsedTime
        t += 1
        print(break_condition)

    if start_time - low_layer_time >= low_layer_loop_time:
        pass

    # elapsedTime = time.perf_counter() - start_time
    # execTimeArr[t] = elapsedTime

    break_condition = all(robots.robotList[i].midSystem.t > simsteps-1 for i in range(robots.numRobots))
    

    

# for t in range(simsteps):
    
#     start_time = time.perf_counter()

#     for i in range(robots.numRobots):
#         robots.robotList[i].midSystem.t = t

#         if robots.robotList[i].midControl.terminate:
#             sequencePair = robots.robotList[i].topSystem.get_sequence_pair()
#             if robots.robotList[i].topSystem.terminate:
#                 xc = np.concatenate([robots.robotList[i].midSystem.xc.flatten()[:2], np.zeros(2)])
#                 print(f"Loop number: {robots.robotList[i].midSystem.t+1}, Robot:{i+1}, Xc: {xc}")
#                 robots.robotList[i].midSystem.stateHistory[:, t] = xc
#                 robots.robotList[i].midSystem.inputHistory[:, t] = np.zeros(2)
#                 # print(f"Loop number: {robots.robotList[i].midSystem.t+1}, Robot:{i+1}, Xc: {robots.robotList[i].midSystem.stateHistory[:2, robots.robotList[i].midSystem.t]}")
#                 continue
#             else:
#                 highLevelInput = robots.find_high_level_input(sequencePair)
#                 robots.robotList[i].midControl.get_constraints(highLevelInput)
#                 robots.robotList[i].midSystem.xf = highLevelInput[1].center
#                 robots.robotList[i].midControl.terminate = False
#         # print(f"Loop number: {robots.robotList[i].midSystem.t+1}, Robot:{i+1}")
#         robots.robotList[i].midSystem.get_mpc(robots.robotList[i].midControl)

#     elapsedTime = time.perf_counter() - start_time
#     execTimeArr[t] = elapsedTime

robotData = {}
for i in range(robots.numRobots):
    stateHistory = robots.robotList[i].midSystem.stateHistory
    inputHistory = robots.robotList[i].midSystem.inputHistory
    robotData[f'stateHistory_{i}'] = stateHistory
    robotData[f'inputHistory_{i}'] = inputHistory

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

df = pd.DataFrame(execTimeArr, columns=["Execution Time"])
df.to_csv("execTimeArr.csv", index=False)
