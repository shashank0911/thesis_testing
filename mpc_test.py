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
target = np.array([25, 12, 0, 0])

# Constraints
xmin = np.array([0, 0, -30, -30])
xmax = np.array([30, 15, 30, 30])
umin = np.array([-5, -5])
umax = np.array([5, 5])

# Define cost matrices
Q = np.diag([10, 10, 1, 1])  # Penalize only the position states
R = np.eye(2)
Qf = Q

# To store the results
state_history = []
input_history = []

# Initialize the current state
x_current = x0

# Loop over time steps
for t in range(0, 250):  # Assuming a total of 250 time steps
    # Define optimization variables
    x = cp.Variable((4, N+1))
    u = cp.Variable((2, N))

    # Define the cost and constraints
    cost = 0
    constraints = [x[:, 0] == x_current]  # Initial state constraint
    
    for k in range(N):
        cost += cp.quad_form(x[:, k] - target, Q) + cp.quad_form(u[:, k], R)
        constraints += [x[:, k+1] == A @ x[:, k] + B @ u[:, k]]
        constraints += [xmin <= x[:, k], x[:, k] <= xmax]
        constraints += [umin <= u[:, k], u[:, k] <= umax]

    # Terminal cost and constraints
    cost += cp.quad_form(x[:, N] - target, Qf)
    constraints += [xmin <= x[:, N], x[:, N] <= xmax]
    constraints += [15 <= x[0, N], x[0, N] <= 30]  # Terminal position constraints
    constraints += [0 <= x[1, N], x[1, N] <= 15]

    # Solve the optimization problem
    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve()

    # Apply the first control input
    u_applied = u.value[:, 0]
    input_history.append(u_applied)

    # Update the state using the system dynamics
    x_current = A @ x_current + B @ u_applied
    state_history.append(x_current)

    # Check if the robot is sufficiently close to the target
    if np.linalg.norm(x_current[:2] - target[:2]) < 0.1:  # Tolerance for stopping
        print("Target reached!")
        break

# Convert lists to arrays for easy handling
state_history = np.array(state_history)
input_history = np.array(input_history)

# Store data in files
np.savetxt("optimal_x_receding.csv", state_history, delimiter=",")
np.savetxt("optimal_u_receding.csv", input_history, delimiter=",")

print("Optimal control inputs and states stored in files.")
