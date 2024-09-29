import cvxpy as cp
import numpy as np
import polytope as pc
import pypolycontain as pp

T = 0.1
N = 25
A = np.array([[1, 0, T, 0],
              [0, 1, 0, T],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

B = np.array([[0, 0],
              [0, 0],
              [1, 0],
              [0, 1]])

num_cols = 2
num_rows = 2

state_min = np.array([0, 0, -30, -30])
state_max = np.array([60, 60, 30, 30])

xmin = 0
xmax = 60
ymin = 0
ymax = 60
vxmin = -30
vxmax = 30
vymin = -30
vymax = 30

Hx = np.vstack([np.eye(4), -np.eye(4)])
hx = np.concatenate((state_max, -state_min))
# hx = np.array([xmax, ymax, vxmax, vymax, -xmin, -ymin, -vxmin, -vymin])

# umin = np.array([-5, -5])
# umax = np.array([5, 5])

uxmin = -5
uxmax = 5
uymin = -5
uymax = 5

Hu = np.vstack([np.eye(2), -np.eye(2)])
hu = np.array([uxmax, uymax, -uxmin, -uymin])

Q = np.diag([10, 10, 1, 1])  
R = np.diag([0.1, 0.1])
Qf = Q

class Robot:
    def __init__(self, id, x0):
        self.id = id
        # self.t = t
        self.terminal_flag = True
        self.x0 = x0
        self.xc = x0
        self.xf = np.array([0, 0, 0, 0])
        self.uc = np.array([0, 0])
        # self.xtmin = np.array([0, 0, 0, 0])
        # self.xtmax = np.array([0, 0, 0, 0])
        self.Hxt = np.vstack([np.eye(4), -np.eye(4)])
        self.hxt = np.concatenate((state_max, -state_min))

        self.Q = Q
        self.R = R
        self.Qf = Qf

        self.x_history = []
        self.u_history = []
        
        # self.x_meas = []
        # self.u_meas = []

        self.transition_sequence = []
        self.sequence_counter = 0
        self.sequence_completion = False

    def find_control_invariant_set(self, cell):
        pass

    # modify to include cur_cell U next_cell set constraint
    def init_mpc(self, xf, xtmin, xtmax):
        self.x0 = self.xc
        # self.xc = x0
        self.xf = np.array([xf[0], xf[1], self.xc[2], self.xc[3]])

        self.hxt[0] = xtmax[0]
        self.hxt[1] = xtmax[1]
        self.hxt[4] = -xtmin[0]
        self.hxt[5] = -xtmin[1]

        # xtmin_array = np.array([xtmin[0], xtmin[1], state_min[2], state_min[3]])
        # xtmax_array = np.array([xtmax[0], xtmax[1], state_max[2], state_max[3]])
        # self.xtmin = xtmin_array
        # self.xtmax = xtmax_array

        # self.x_meas = []
        # self.u_meas = []


    def run_mpc(self):
        x = cp.Variable((4, N+1))
        u = cp.Variable((2, N))

        cost = 0
        constraints = [x[:, 0] == self.xc]

        for k in range(N):
            cost += cp.quad_form(x[:, k] - self.xf, self.Q) + cp.quad_form(u[:, k], self.R)
            constraints += [x[:, k+1] == A @ x[:, k] + B @ u[:, k]]
            # constraints += [state_min <= x[:, k], x[:, k] <= state_max]
            # constraints += [umin <= u[:, k], u[:, k] <= umax]
            constraints += [Hx @ x[:, k] <= hx]
            constraints += [Hu @ u[:, k] <= hu]

        cost += cp.quad_form(x[:, N] - self.xf, Qf)
        # constraints += [state_min <= x[:, N], x[:, N] <= state_max]
        constraints += [self.Hxt @ x[:, N] <= self.hxt] 

        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()

        self.uc = u.value[:, 0]
        self.u_history.append(self.uc)

        self.xc = A @ self.xc + B @ self.uc
        self.x_history.append(self.xc)

        if np.linalg.norm(self.xc[:2] - self.xf[:2]) < 7.5:
            self.terminal_flag = True

    def save_data(self):
        self.x_history = np.array(self.x_history)
        self.u_history = np.array(self.u_history)
        np.savetxt("optimal_x_receding.csv", self.x_history, delimiter=",")
        np.savetxt("optimal_u_receding.csv", self.u_history, delimiter=",")
        print("Data for robot ", self.id, " is saved")


class Robots:
    def __init__(self, id_list, x0_list):

        # Id is a list of strings
        self.id_list = id_list
        self.num = len(id_list)

        self.t = 0

        # dataype array
        self.x0_list = x0_list
        self.xc_list = x0_list

        self.robots = []
        self.env = Environment()
        self.env.partition_environment()


    def add_robots(self):
        for i in range(0, self.num):
            x0 = np.array(self.x0_list[:, i])
            self.robots.append(Robot(self.id_list[i], x0))

    # Motion planning using LTL
    def motion_plan(self):
        # initialize transition sequence for each robot after running ltl sequence
        for i in range(0, self.num):
            self.robots[i].transition_sequence = ["cell_11", "cell_12", "cell_22"]
        
    # returns cur_cell and next_cell index
    def get_next_transition(self, i):

        # if transitions are repeated, change sequence counter to reflect that
        cur_cell = self.robots[i].transition_sequence[self.robots[i].sequence_counter]
        self.robots[i].sequence_counter += 1
        if self.robots[i].sequence_completion:
            next_cell = "None"
        else:
            if self.robots[i].sequence_counter >= len(self.robots[i].transition_sequence):
                next_cell = "None"
                self.robots[i].sequence_completion = True
            else:
                next_cell = self.robots[i].transition_sequence[self.robots[i].sequence_counter]
            # next_cell = f"cell_11"
        return (cur_cell, next_cell)
        
    def trajectory_plan(self):
        # When robot has not reached terminal state in terminal set
        for i in range(0, self.num):
            if not self.robots[i].terminal_flag:
                self.robots[i].run_mpc()
                self.xc_list[:, i] = self.robots[i].xc

            # Robot has reached terminal state in terminal cell; transition to next cell
            elif self.robots[i].terminal_flag:
                # change later based on motion plan
                # modify to include cur_cell U next_cell and send it to init_mpc
                if not self.robots[i].sequence_completion:
                    (cur_cell, next_cell) = self.get_next_transition(i)
                    if next_cell in self.env.cell_dict:
                        print("Robot will now move to ", next_cell, '\n')
                        self.robots[i].terminal_flag = False

                        # Initialize final position as centre of the cell
                        xtmin = self.env.cell_dict[next_cell][0]
                        xtmax = self.env.cell_dict[next_cell][1]
                        xf1 = xtmin[0] + (xtmax[0] - xtmin[0]) / 2
                        xf2 = xtmin[1] + (xtmax[1] - xtmin[1]) / 2
                        xf = np.array([xf1, xf2])
                        
                        self.robots[i].init_mpc(xf, xtmin, xtmax)
                        self.robots[i].run_mpc()
                        self.xc_list[:, i] = self.robots[i].xc
                else:
                    # If next cell is not found, robot will stop as flag is not reset
                    print("Next cell in the sequence is not found for robot with id: ", i + 1, '\n')
                    # self.robots[i].init_mpc(self.robots[i].xc, self.robots[i].xf)
        self.t += 1

class Environment:
    def __init__(self, cols = 2, rows = 2):
        self.boundary = [(hx[4], hx[5]), (hx[0], hx[1])]
        # self.cell_array = []
        self.cell_dict = {}

        # Change num_cols and num_rows to cols and rows if needed
        self.num_cols = num_cols
        self.num_rows = num_rows

    def partition_environment(self):
        cell_width = (hx[0] - hx[4])/self.num_cols
        cell_height = (hx[1] - hx[5])/self.num_rows
        # self.cell_array = [[None for _ in range(self.num_cols)] for _ in range(num_rows)]
        
        for i in range(num_rows):
            for j in range(num_cols):
                cell_xmin = self.boundary[0][0] + j * cell_width
                cell_xmax = cell_xmin + cell_width
                cell_ymax = self.boundary[1][1] - i * cell_height
                cell_ymin = cell_ymax - cell_height
                # self.cell_array[i][j] = [(cell_xmin, cell_ymin), (cell_xmax,cell_ymax)]
                cell_key = f"cell_{i+1}{j+1}"
                self.cell_dict[cell_key] = [(cell_xmin, cell_ymin), (cell_xmax, cell_ymax)]
        

if __name__ == "__main__":
    id_list = ["3735"]
    x0_list = np.array([5, 55, 0, 0])
    x0_list = x0_list[:, np.newaxis]
    robots = Robots(id_list, x0_list)
    robots.add_robots()
    robots.motion_plan()
    for t in range (0,100):
        robots.trajectory_plan()
    for i in range(0, robots.num):
        robots.robots[i].save_data()

    print("Program done")
