import numpy as np
from scipy.integrate import odeint
from scipy.linalg import solve_continuous_lyapunov
import sympy as sp

class LowLevelSystem:
    def __init__(self, x, y, theta, t_vals):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.1
        self.omega = 0

        self.t = 0
        self.k = -1
        # Sampling time of mid level system
        self.t_vals = t_vals
        self.ct = 0

        self.stateHistory = np.zeros((5, t_vals.simsteps * t_vals.low_steps))
        self.inputHistory = np.zeros((2, t_vals.simsteps * t_vals.low_steps))
        self.trajectoryHistory = np.zeros((2, t_vals.simsteps * t_vals.low_steps))

        # self.a = 0
        # self.alpha = 0
    
    def update_state(self, lowControl, xm):
        # Update k when midlevel goes to next step/loop
        # self.k = int(self.t/self.T)
        
        sk = sp.Matrix(xm[4:6, 0].reshape(-1, 1))
        vk = sp.Matrix(xm[6:8, 0].reshape(-1, 1))
        vkplus = sp.Matrix(xm[10:12, 0].reshape(-1, 1))
        T = self.t_vals.T_low

        a, alpha = lowControl.fbl_control(self, sk, vk, vkplus)
        self.v += a * T
        self.omega += alpha * T 
        self.theta += self.omega * T
        self.theta %= (2 * np.pi)
        self.x += self.v * np.cos(self.theta) * T
        self.y += self.v * np.sin(self.theta) * T
        # print("u: ", np.array([a*T, alpha*T]))
        # self.stateHistory.append(np.array([[self.x], [self.y], [self.theta], [self.v], [self.omega]]))
        # self.inputHistory.append(np.array([[a], [alpha]]))
        self.stateHistory[:, self.ct] = np.array([[self.x], [self.y], [self.theta], [self.v], [self.omega]]).flatten()
        self.inputHistory[:, self.ct] = np.array([[a], [alpha]]).flatten()



class LowLevelControl:
    def __init__(self, Kp, Kd, sigma):
        self.Kp = Kp
        self.Kd = Kd
        self.sigma = sigma
        # self.alpha = alpha
        # self.delta = (2 - self.beta) * self.T

        self.t, self.k, self.T = sp.symbols('t k T') 
        self.sk = sp.MatrixSymbol('sk', 2, 1)
        self.vk = sp.MatrixSymbol('vk', 2, 1)
        self.vkplus = sp.MatrixSymbol('vkplus', 2, 1)
        # self.sk, self.vk, self.vkplus = sp.symbols('sk vk vkplus')
        
        self.xd = self.xd_symbolic()
        # self.ym = self.xd_symbolic()
        # self.xd = sp.Matrix([[self.xm], [self.ym]])
        
        self.xd_dot = sp.diff(self.xd, self.t)
        self.xd_2dot = sp.diff(self.xd_dot, self.t)
        self.xd_3dot = sp.diff(self.xd_2dot, self.t)
        # print(self.xd)
        # print('\n')
        # print(self.xd_dot)
        # print('\n')
        # print(self.xd_2dot)
        # print('\n')
        # print(self.xd_3dot)
        # print('\n')

        A = np.block([[np.zeros((2, 2)), np.eye(2)], [-self.Kp, -self.Kd]])
        Q = np.eye(4)
        self.P = solve_continuous_lyapunov(A.T, -Q)

        # self.trajectoryHistory = []

        
    def xd_symbolic(self):
        # t = self.t
        # k = self.k
        # T = self.T
        # sk = self.sk
        # vk = self.vk
        # vkplus = self.vk
        
        t_p1 = self.t - self.k*self.T - self.T/2
        t_p2 = self.t - self.k*self.T - self.T/2

        x1 = (self.sk + self.vk * (self.t - self.k*self.T) - 
            (self.vkplus - self.vk) / self.T**2 * t_p1**3 * (1 + 2*t_p1/self.T))
        x2 = (self.sk + self.vk * (self.t - self.k*self.T) -
            (self.vkplus - self.vk) / self.T**2 * t_p2**3 * (1 - 2*t_p2/self.T))
        xd = sp.Piecewise(
            (x1, sp.And(self.t >= self.k*self.T, self.t < (self.k+0.5)*self.T)),
            (x2, sp.And(self.t >= (self.k+0.5)*self.T, self.t < (self.k+1)*self.T))
        ) 

        return xd
    
    def fbl_control(self, lowSystem, sk, vk, vkplus):
        x = lowSystem.x
        y = lowSystem.y
        theta = lowSystem.theta
        v = lowSystem.v
        omega = lowSystem.omega

        t = lowSystem.t
        k = lowSystem.k
        T = lowSystem.t_vals.T_mid
        print("t:", t, ", k:", k)

        xl = np.array([[x], [y]])
        # print(np.shape(xl))

        ctheta = np.cos(theta)
        stheta = np.sin(theta)

        xd = np.array(self.xd.subs({self.t: t, self.k: k, self.T: T, self.sk: sk, self.vk: vk, self.vkplus: vkplus})).astype(float)
        xd_dot = np.array(self.xd_dot.subs({self.t: t, self.k: k, self.T: T, self.sk: sk, self.vk: vk, self.vkplus: vkplus})).astype(float)
        xd_2dot = np.array(self.xd_2dot.subs({self.t: t, self.k: k, self.T: T, self.sk: sk, self.vk: vk, self.vkplus: vkplus})).astype(float)
        xd_3dot = np.array(self.xd_3dot.subs({self.t: t, self.k: k, self.T: T, self.sk: sk, self.vk: vk, self.vkplus: vkplus})).astype(float)
        # print(np.shape(xd_dot))
        print("xd:", xd.T, ", xd_dot:", xd_dot.T, ", xd_2dot:", xd_2dot.T, ", xd_3dot:", xd_3dot.T)

        e = xl - xd
        # print("xl: ", xl.flatten(), "xd: ", xd.flatten())
        # print("sk: ", sk, "vk: ", vk, "vkplus: ", vkplus)
        e_dot = np.array([[v*ctheta], [v*stheta]]) - xd_dot
        e_2dot = - self.Kp @ e - self.Kd @ e_dot
        ed_2dot = xd_2dot - self.Kp @ e - self.Kd @ e_dot
        
        ad = np.array([[ctheta, stheta]]) @ ed_2dot
        ad = ad.item()
        omegad = 1/v * np.array([[-stheta, ctheta]]) @ ed_2dot
        omegad = omegad.item()
        omegad_dot = (np.array([[ad/v**2 * stheta - omega/v * ctheta, -ad/v**2 * ctheta - omega/v * stheta]]) @ ed_2dot + 
                      1/v * np.array([[-stheta, ctheta]]) @ (xd_3dot - self.Kp @ e_dot - self.Kd @ e_2dot))
        omegad_dot = omegad_dot.item()
        alphad = -0.5 * self.sigma * (omega - omegad) + omegad_dot - 2 * np.hstack((e.T, e_dot.T)) @ self.P @ np.array([[0], [0], [-v*stheta], [v*ctheta]])
        alphad = alphad.item()
        print("x:", x, ", y:", y, ", theta:", theta, ", v:", v, "\nvc:", v*ctheta, ", vs:", v*stheta, ", ad:", ad)
        print("omega:", omega, ", omegad:", omegad, ", omegad_dot", omegad_dot, ", alphad:", alphad)
        print("e:", e.T, ", e_dot:", e_dot.T, ", e_2dot:", e_2dot.T, ", ed_2dot:", ed_2dot.T, '\n')
        
        
        u = np.array([[ad], [alphad]])
        # print("u: ", u)
        # print('\n')
        # print(np.shape(xd))
        # lowSystem.trajectoryHistory.append(xd)
        lowSystem.trajectoryHistory[:, lowSystem.ct] = xd.flatten()
        
        return ad, alphad
