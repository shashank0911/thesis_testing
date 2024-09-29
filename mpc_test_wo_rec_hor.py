import cvxpy as cp
import numpy as np

# System parameters
T = 0.1  # Sampling time
N = 25   # Prediction horizon

# Matrices defining the dynamics
A = np.array([[1, 0, T, 0],
              [0, 1, 0, T],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

B = np.array([[0, 0],
              [0, 0],
              [1, 0],
              [0, 1]])

# Initial state and target state
x0 = np.array([1, 2, 0, 0])
target = np.array([25, 10, 0, 0])

# Constraints
xmin = np.array([0, 0, -30, -30])
xmax = np.array([30, 15, 30, 30])
umin = np.array([-5, -5])
umax = np.array([5, 5])

# Define optimization variables
x = cp.Variable((4, N+1))
u = cp.Variable((2, N))

# Define cost matrices
Q = np.diag([10, 10, 1, 1])
R = np.eye(2)
Qf = Q

# Cost function
cost = 0
constraints = []

for k in range(N):
    cost += cp.quad_form(x[:, k] - target, Q) + cp.quad_form(u[:, k], R)
    constraints += [x[:, k+1] == A @ x[:, k] + B @ u[:, k]]
    constraints += [xmin <= x[:, k], x[:, k] <= xmax]
    constraints += [umin <= u[:, k], u[:, k] <= umax]

# Terminal cost
cost += cp.quad_form(x[:, N] - target, Qf)
constraints += [xmin <= x[:, N], x[:, N] <= xmax]
constraints += [15 <= x[0, N], x[0, N] <= 30]  # Terminal position constraints
constraints += [0 <= x[1, N], x[1, N] <= 15]

# Initial state constraint
constraints += [x[:, 0] == x0]

# Define and solve the optimization problem
problem = cp.Problem(cp.Minimize(cost), constraints)
problem.solve()

# Extract the optimal control inputs and states
optimal_u = u.value
optimal_x = x.value

# Store data in files
np.savetxt("optimal_x.csv", optimal_x.T, delimiter=",")
np.savetxt("optimal_u.csv", optimal_u.T, delimiter=",")

print("Optimal control inputs and states stored in files.")
