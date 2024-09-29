import numpy as np
from scipy.integrate import odeint
from scipy.linalg import solve_continuous_lyapunov
import sympy as sp

class LowLevelSystem:
    def __init__(self, x, y, theta, t_vals):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0
        self.omega = 0

        self.t = 0
        self.k = 0
        # Sampling time of mid level system
        self.t_vals = t_vals

        self.stateHistory = np.zeros((5, 61))
        self.inputHistory = np.zeros((2, 60))

        # self.a = 0
        # self.alpha = 0
    
    def update_state(self, lowControl, xm):
        # Update k when midlevel goes to next step/loop
        # self.k = int(self.t/self.T)
        
        sk = xm[4:6, 0].reshape(-1, 1)
        vk = xm[6:8, 0].reshape(-1, 1)
        vkplus = xm[10:12, 0].reshape(-1, 1)

        a, alpha = lowControl.fbl_control(self, sk, vk, vkplus)
        self.v += a
        self.omega += alpha 
        self.theta += self.omega
        self.x += self.v * np.cos(self.theta)
        self.y += self.v * np.sin(self.theta)




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

        A = np.block([[np.zeros((2, 2)), np.eye(2)], [-self.Kp, -self.Kd]])
        Q = np.eye(4)
        self.P = solve_continuous_lyapunov(A.T, -Q)

        self.trajectoryHistory = []

        
    def xd_symbolic(self):
        t = self.t
        k = self.k
        T = self.T
        sk = self.sk
        # skplus = self.skplus
        vk = self.vk
        vkplus = self.vk
        
        t_p1 = t - k*T - T/2
        t_p2 = t - k*T - T/2

        x1 = (sk + vk * (t - k*T) - 
            (vkplus - vk) / T**2 * t_p1**3 * (1 + 2*t_p1/T))
        x2 = (sk + vk * (t - k*T) -
            (vkplus - vk) / T**2 * t_p2**3 * (1 - 2*t_p2/T))

        xd = sp.Piecewise(
            (x1, sp.And(t >= k*T, t < (k+0.5)*T)),
            (x2, sp.And(t >= (k+0.5)*T, t < (k+1)*T))
        ) 

        # tp1 = t - k*self.T - self.alpha*self.T/2
        # # Check if this eqn is correct
        # tp2 = t - k*self.T + self.delta*self.T/2
        # delT = self.delta * self.T
        # x1 = sk[0] + vk[0]*(t - self.T) + (vkplus[0] - vk[0])/(delT * self.T) * tp1**3 * (1 - 2*tp1/delT)
        # x2 = skplus[0] + vkplus[0]*(t - (k+1)*self.T) + (vkplus[0] - vk[0])/(delT * self.T) * tp2**3 * (1 - 2*tp2/delT)
        # y1 = sk[1] + vk[1]*(t - self.T) + (vkplus[1] - vk[1])/(delT * self.T) * tp1**3 * (1 - 2*tp1/delT)
        # y2 = skplus[1] + vkplus[1]*(t - (k+1)*self.T) + (vkplus[1] - vk[1])/(delT * self.T) * tp2**3 * (1 - 2*tp2/delT)

        # xl = sp.Piecewise(
        #     (x1, sp.And(t >= (k+0.5)*self.T, t < (k+1)*self.T)),
        #     (x2, sp.And(t >= (k+1)*self.T, t < (k+1.5)*self.T)),
        # )

        # xd = sp.Piecewise(
        #     (sp.Matrix([[x1], [y1]]), sp.And(t >= (k+0.5)*self.T, t < (k+1)*self.T)),
        #     (sp.Matrix([[x2], [y2]]), sp.And(t >= (k+1)*self.T, t < (k+1.5)*self.T)),
        # )

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

        xl = np.array([[x], [y]])

        ctheta = np.cos(theta)
        stheta = np.sin(theta)

        xd = np.array(self.xd.subs({self.t: t, self.k: k, self.T: T, self.sk: sk, self.vk: vk, self.vkplus: vkplus})).astype(float)
        xd_dot = np.array(self.xd_dot.subs({self.t: t, self.k: k, self.T: T, self.sk: sk, self.vk: vk, self.vkplus: vkplus})).astype(float)
        xd_2dot = np.array(self.xd_2dot.subs({self.t: t, self.k: k, self.T: T, self.sk: sk, self.vk: vk, self.vkplus: vkplus})).astype(float)
        xd_3dot = np.array(self.xd_3dot.subs({self.t: t, self.k: k, self.T: T, self.sk: sk, self.vk: vk, self.vkplus: vkplus})).astype(float)

        e = xl - xd
        e_dot = np.array([[v*ctheta], [v*stheta]]) - xd_dot
        e_2dot = - self.Kp*e - self.Kd*e_dot
        ed_2dot = xd_2dot - self.Kp*e - self.Kd*e_dot
        
        ad = np.array([[ctheta, stheta]]) * ed_2dot
        omegad = 1/v * np.array([[-stheta, ctheta]]) * ed_2dot
        omegad_dot = (np.array([[ad/v**2 * stheta - omega/v * ctheta, -ad/v**2 * ctheta - omega/v * stheta]]) * ed_2dot + 
                      1/v * np.array([[-stheta, ctheta]]) * (xd_3dot - self.Kp * e_dot - self.Kd * e_2dot))
        alphad = -0.5 * self.sigma * (omega - omegad) + omegad_dot - 2 * np.array([[e.T, e_dot.T]]) * self.P * np.array([[0], [0], [-v*stheta], [v*ctheta]])
        
        u = np.array([[ad], [alphad]])

        self.trajectoryHistory.append(xd)
        
        return ad, alphad


        
    
         

          