import numpy as np
import cvxpy as cp
from scipy.linalg import block_diag
from polytope import Polytope, bounding_box_to_polytope, bounding_box

class DiscreteSystem:
    def __init__(self, A, B, C):
        self.A = A
        self.B = B
        self.C = C

class Dimensions:
    def __init__(self, nx, nu, ny, N, simsteps):
        self.nx = nx
        self.nu = nu
        self.ny = ny
        self.N = N
        self.simsteps = simsteps

class Weight:
    def __init__(self, Q, R, Qf):
        self.Q = Q
        self.R = R
        self.Qf = Qf

class Time:
    def __init__(self, T_mid, low_steps):
        self.T_mid = T_mid
        self.low_steps = low_steps
        self.T_low = self.T_mid / self.low_steps
        self.T_low_arr = np.linspace(0, self.T_mid, self.low_steps+1)

        

class MidLevelSystem:
    def __init__(self, id, x0, T, dim):
        A = np.block([[np.eye(2), T * np.eye(2)], [np.zeros((2, 2)), np.eye(2)]])
        B = np.block([[np.zeros((2, 2))], [np.eye(2)]])
        C = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.sys = DiscreteSystem(A, B, C)

        self.id = id
        self.t = 0
        self.T = T

        self.x0 = x0
        self.xc = x0
        self.xf = x0[:2]
        self.uc = np.zeros((dim.nu * dim.N, 1))
        self.yc = np.zeros((dim.nx * (dim.N+1), 1))
        self.dim = dim

        self.predmodgen()
        self.stateHistory = np.zeros((dim.nx, dim.simsteps))
        self.inputHistory = np.zeros((dim.nu, dim.simsteps))
        # self.stateHistory[:, 0] = x0

    def predmodgen(self):
        nx = self.dim.nx
        nu = self.dim.nu
        N = self.dim.N

        T = np.zeros((nx * (N + 1), nx))
        for k in range(N + 1):
            T[k*nx:(k+1)*nx, :] = np.linalg.matrix_power(self.sys.A, k)

        S = np.zeros((nx * (N + 1), nu * N))
        for k in range(1, N + 1):
            for i in range(k):
                S[k*nx:(k+1)*nx, i*nu:(i+1)*nu] = np.dot(np.linalg.matrix_power(self.sys.A, k-1-i), self.sys.B)

        self.predmod = {'T': T, 'S': S}

    def get_mpc(self, midControl):
        midControl.run_mpc2(self)
        self.uc = midControl.uc
        self.yc = self.predmod['T'] @ self.xc.reshape(-1, 1) + self.predmod['S'] @ self.uc
        # print(self.yc.T)
        self.xc = self.sys.A @ self.xc.reshape(-1, 1) + self.sys.B @ self.uc[:self.dim.nu]
        

        pt = self.xc[:2].reshape(-1, 1)
        if midControl.terminalPosConstraints.contains(pt).all():
            print("We are inside!\t")
            print(f"Robot: {self.id}, counter: {midControl.counter}")
            midControl.terminate = True
            midControl.counter = 0
        else:
            if midControl.counter == self.dim.N:
                midControl.counter = 0
                print(f"Target cell not reached in N steps by robot {self.id}")
            else:
                midControl.counter += 1
        self.inputHistory[:, self.t] = self.uc[:self.dim.nu].flatten()
        self.stateHistory[:, self.t] = self.xc.flatten()
        # print(f"Loop number: {self.t+1}, Robot:{self.id}, Xc:{self.stateHistory[:2, self.t]}")

    def F_2(self):
        pass

    def F_1inv(self):
        pass
        

class MidLevelControl:
    def __init__(self, weight, velocityConstraints, inputConstraints, midSystem):
        self.uc = np.zeros((midSystem.dim.nu*midSystem.dim.N, 1))
        self.weight = weight
        self.positionConstraints = None
        self.terminalConstraints = None
        self.terminalPosConstraints = None
        self.stateConstraints = None
        self.velocityConstraints = velocityConstraints
        self.inputConstraints = inputConstraints
        self.costgen(midSystem)
        self.counter = 0
        self.terminate = True

    def costgen(self, midSystem):
        N = midSystem.dim.N
        nx = midSystem.dim.nx
        nu = midSystem.dim.nu
        Qbar = block_diag(np.kron(np.eye(N), self.weight.Q), self.weight.Qf)
        Rbar = np.kron(np.eye(N), self.weight.R)
        H = midSystem.predmod['S'].T @ Qbar @ midSystem.predmod['S'] + Rbar
        hx0 = midSystem.predmod['S'].T @ Qbar @ midSystem.predmod['T']
        hxref = -midSystem.predmod['S'].T @ Qbar @ np.kron(np.ones((N+1, 1)), np.eye(nx))
        huref = -Rbar @ np.kron(np.ones((N, 1)), np.eye(nu))
        h = np.hstack([hx0, hxref, huref])
        self.costmat = {'H': H, 'h': h}

    def get_constraints(self, highLevelInput):
        currentCell, terminalCell = highLevelInput
        U = currentCell.positionConstraints.union(terminalCell.positionConstraints)
        lb, ub = bounding_box(U)
        self.positionConstraints = bounding_box_to_polytope(lb, ub)

        # print(self.positionConstraints.list_poly)
        self.terminalConstraints = terminalCell.controlInvariantSet
        self.terminalPosConstraints = terminalCell.controlInvariantPosSet
        A = block_diag(self.positionConstraints.A, self.velocityConstraints.A)
        b = np.concatenate((self.positionConstraints.b, self.velocityConstraints.b), axis=None)
        self.stateConstraints = Polytope(A, b)
        # print(self.stateConstraints)

    def run_mpc2(self, midSystem):
        dim = midSystem.dim

        x0 = midSystem.xc
        xf = midSystem.xf

        u = cp.Variable((dim.nu*dim.N, 1))

        if self.counter == dim.N:
            combinedStateConstraints_A = np.kron(np.eye(dim.N + 1), self.terminalConstraints.A)
            combinedStateConstraints_b = np.tile(self.terminalConstraints.b, self.counter + 1)
        else: 
            modifiedStateConstraints_A = np.kron(np.eye(dim.N - self.counter), self.stateConstraints.A)
            modifiedStateConstraints_b = np.tile(self.stateConstraints.b, dim.N - self.counter)
            modifiedTerminalConstraints_A = np.kron(np.eye(self.counter + 1), self.terminalConstraints.A)
            modifiedTerminalConstraints_b = np.tile(self.terminalConstraints.b, self.counter + 1)
            combinedStateConstraints_A = block_diag(modifiedStateConstraints_A, modifiedTerminalConstraints_A)
            combinedStateConstraints_b = np.concatenate([modifiedStateConstraints_b, modifiedTerminalConstraints_b])

        combinedInputConstraints_A = np.kron(np.eye(dim.N), self.inputConstraints.A)
        combinedInputConstraints_b = np.tile(self.inputConstraints.b, dim.N)
        combinedStateEquation = midSystem.predmod['T'] @ x0.reshape(-1,1) + midSystem.predmod['S'] @ u

        constraints = [combinedInputConstraints_A @ u <= combinedInputConstraints_b.reshape(-1,1),
                       combinedStateConstraints_A @ combinedStateEquation <= combinedStateConstraints_b.reshape(-1,1)]
        ur = np.array([0, 0])
        objective = 0.5 * cp.quad_form(u, self.costmat['H']) + (self.costmat['h'] @ np.concatenate([midSystem.xc.flatten(), xf, ur, ur])).T @ u
        
        prob = cp.Problem(cp.Minimize(objective), constraints)
        prob.solve()

        if prob.status == cp.OPTIMAL:
            self.uc = u.value
        else:
            print(f"Optimization problem failed to solve for Robot: {midSystem.id}, counter: {self.counter}")        
            self.uc = np.zeros((midSystem.dim.nu*midSystem.dim.N, 1))
        
        






        
