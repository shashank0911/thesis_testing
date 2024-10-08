import pickle
import numpy as np
import matplotlib.pyplot as plt
from polytope import Polytope

# with open('robot_cell_data.pkl', 'rb') as f:
#     data = pickle.load(f)

# robotData = data['robot_data']
# cellData = data['cell_data']
# stateHistory = robotData[f'stateHistory_{0}']

# x_data = stateHistory[0, :]
# y_data = stateHistory[1, :]
# print(x_data)
# print(y_data)


# x = np.array([True, False, False])
# print(x.all())

theta = -np.pi/2
theta = (theta + np.pi) % (2 * np.pi) - np.pi
print(theta)




