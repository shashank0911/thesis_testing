import pickle
import numpy as np
import matplotlib.pyplot as plt
from polytope import Polytope

with open('robot_cell_data.pkl', 'rb') as f:
    data = pickle.load(f)

robotData = data['robot_data']
cellData = data['cell_data']

fig1, ax1 = plt.subplots(figsize=(10, 6))

for i in range(robotData[f'numRobots']):
    stateHistoryMid = robotData[f'stateHistoryMid_{i}']

    x_data = stateHistoryMid[0, :]
    y_data = stateHistoryMid[1, :]

    random_color = np.random.rand(3,)
    ax1.plot(x_data, y_data, '-', color=random_color, linewidth=2, markersize=10, marker='.', label=f'Robot {i+1}')
    ax1.set_xlabel("X coordinate")
    ax1.set_ylabel("Y coordinate")
    ax1.set_title("MAS evolution")

for key in cellData:
    constSet_A = cellData[key]['constraintPos_A']
    constSet_b = cellData[key]['constraintPos_b']
    invSet_A = cellData[key]['invSet_A']
    invSet_b = cellData[key]['invSet_b']
    constSet = Polytope(constSet_A, constSet_b)
    invSet = Polytope(invSet_A, invSet_b)
    constSet.plot(ax=ax1, color=[0.8, 0.8, 0.8], alpha=0.3, linewidth=1.5, edgecolor='k')
    invSet.plot(ax=ax1, color=[0.6, 0.6, 0.6], alpha=0.3, linewidth=1.5, edgecolor='k')

plt.grid(True)
plt.axis('auto')
plt.legend(loc='upper right')
plt.savefig("evol_ideal.png")
# plt.show()

fig2, ax2 = plt.subplots(figsize=(10, 6))

for i in range(robotData[f'numRobots']):
    stateHistoryMid = robotData[f'stateHistoryMid_{i}']
    stateHistoryLow = robotData[f'stateHistoryLow_{i}']
    trajectoryHistoryLow = robotData[f'trajectoryHistoryLow_{i}']

    x_data_mid = stateHistoryMid[0, :]
    y_data_mid = stateHistoryMid[1, :]
    x_data_low = stateHistoryLow[0, :]
    y_data_low = stateHistoryLow[1, :]
    x_data_inter = trajectoryHistoryLow[0, :]
    y_data_inter = trajectoryHistoryLow[1, :]
    # print(x_data_mid)
    # print(x_data_inter)
    # print(x_data_low)

    random_color = np.random.rand(3,)
    random_color_2 = np.random.rand(3,)
    ax2.scatter(x_data_mid, y_data_mid, color=random_color, label=f'Points from middle layer')
    # ax2.plot(x_data_mid, y_data_mid, '-', color=random_color, linewidth=2, label=f'Points from middle layer')
    ax2.plot(x_data_inter, y_data_inter, '-', color=random_color_2, linewidth=1.5, label=f'Interpolation')
    ax2.plot(x_data_low, y_data_low, '-', color=random_color, linewidth=1.5, label=f'Robot position')
    ax2.set_xlabel("X coordinate")
    ax2.set_ylabel("Y coordinate")
    ax2.set_title("MAS evolution")


for key in cellData:
    constSet_A = cellData[key]['constraintPos_A']
    constSet_b = cellData[key]['constraintPos_b']
    invSet_A = cellData[key]['invSet_A']
    invSet_b = cellData[key]['invSet_b']
    constSet = Polytope(constSet_A, constSet_b)
    invSet = Polytope(invSet_A, invSet_b)
    constSet.plot(ax=ax2, color=[0.8, 0.8, 0.8], alpha=0.3, linewidth=1.5, edgecolor='k')
    invSet.plot(ax=ax2, color=[0.6, 0.6, 0.6], alpha=0.3, linewidth=1.5, edgecolor='k')

plt.grid(True)
plt.axis('auto')
plt.legend(loc='upper right')
plt.savefig("evol_true.png")
plt.show()


# for i in range(len([key for key in robotData.keys() if 'stateHistory' in key])):
#     stateHistory = robotData[f'stateHistory_{i}']
#     inputHistory = robotData[f'inputHistory_{i}']

#     vx_data = stateHistory[2, :]
#     vy_data = stateHistory[3, :]
#     dvx_data = inputHistory[0, :]
#     dvy_data = inputHistory[1, :]

#     time_steps = np.arange(stateHistory.shape[1])
    
#     fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))

#     ax1.step(time_steps, vx_data, '-', color=random_color, linewidth=2)
#     ax1.set_title(f'Robot {i+1} - Velocity along X (State)')
#     ax1.set_xlabel('Time')
#     ax1.set_ylabel('Velocity X')
#     ax1.grid(True)

#     ax2.step(time_steps, vy_data, '-', color=random_color, linewidth=2)
#     ax2.set_title(f'Robot {i+1} - Velocity along Y (State)')
#     ax2.set_xlabel('Time')
#     ax2.set_ylabel('Velocity Y')
#     ax2.grid(True)

#     plt.tight_layout()
#     plt.show()

#     time_steps = np.arange(inputHistory.shape[1])

#     fig, (ax3, ax4) = plt.subplots(2, 1, figsize=(10, 6))

#     ax3.step(time_steps, dvx_data, '-', color=random_color, linewidth=2)
#     ax3.set_title(f'Robot {i+1} - Input Velocity along X')
#     ax3.set_xlabel('Time')
#     ax3.set_ylabel('Input Velocity X')
#     ax3.grid(True)

#     ax4.step(time_steps, dvy_data, '-', color=random_color, linewidth=2)
#     ax4.set_title(f'Robot {i+1} - Input Velocity along Y')
#     ax4.set_xlabel('Time')
#     ax4.set_ylabel('Input Velocity Y')
#     ax4.grid(True)

#     plt.tight_layout()
#     plt.show()

