import sympy as sp
import numpy as np
import matplotlib.pyplot as plt
from sympy import init_printing
init_printing() 

t, k, T = sp.symbols('t k T')
s_k, s_k_next, v_k, v_k_next = sp.symbols('s_k s_k_next v_k v_k_next')
# s_k = sp.MatrixSymbol('s_k', 2, 1)
# v_k = sp.MatrixSymbol('v_k', 2, 1)
# v_k_next = sp.MatrixSymbol('v_k_next', 2, 1)

# Modified implementation; doesn't start from midpoint
t_p1 = t - k*T - T/2
t_p2 = t - k*T - T/2
x1 = (s_k + v_k * (t - k*T) + 
      (v_k_next - v_k) / T**2 * t_p1**3 * (1 + 2*t_p1/T))
x2 = (s_k + v_k * (t - k*T) +
      (v_k_next - v_k) / T**2 * t_p2**3 * (1 - 2*t_p2/T)) 
x = sp.Piecewise(
    (x1, sp.And(t >= k*T, t < (k+0.5)*T)),
    (x2, sp.And(t >= (k+0.5)*T, t < (k+1)*T))
)
y1 = (s_k + v_k * (t - k*T) + 
      (v_k_next - v_k) / T**2 * t_p1**3 * (1 + 2*t_p1/T))
y2 = (s_k + v_k * (t - k*T) +
      (v_k_next - v_k) / T**2 * t_p2**3 * (1 - 2*t_p2/T)) 
y = sp.Piecewise(
    (x1, sp.And(t >= k*T, t < (k+0.5)*T)),
    (x2, sp.And(t >= (k+0.5)*T, t < (k+1)*T))
)
# x_dot = sp.diff(x, t)
# x_2dot = sp.diff(x_dot, 2)
# print(x)
n = 15
l = 5
s_vals_x = np.zeros(l)
v_vals_x = np.array([10, 20, 40, 57, 91])
for i in range(1, len(v_vals_x)):
    s_vals_x[i] = s_vals_x[i-1] + v_vals_x[i-1]

s_vals_y = np.zeros(l)
v_vals_y = np.array([10, -10, 10, -10, 10])
for i in range(1, len(v_vals_y)):
    s_vals_y[i] = s_vals_y[i-1] + v_vals_y[i-1]

# print(s_vals_x)
# print(v_vals_x)
t_vals = np.linspace(0, l-1-1/(n), (l-1)*n)
# print(t_vals)
x_vals = []
y_vals = []
T_val = 1

for t_val in t_vals:
    k_val = int(t_val / T_val)
    # if t_val < (k_val+0.5)*T_val:
    #     k_val -= 1
    s_k_val_x = s_vals_x[k_val]
    # s_k_next_val = s_vals_x[k_val + 1]
    v_k_val_x = v_vals_x[k_val]
    v_k_next_val_x = v_vals_x[k_val + 1]

    x_val = x.subs({
            t: t_val,
            k: k_val,
            T: T_val,
            s_k: s_k_val_x,
            v_k: v_k_val_x,
            v_k_next: v_k_next_val_x
        }).evalf()
    
    x_vals.append(float(x_val))

    s_k_val_y = s_vals_y[k_val]
    # s_k_next_val = s_vals_x[k_val + 1]
    v_k_val_y = v_vals_y[k_val]
    v_k_next_val_y = v_vals_y[k_val + 1]

    y_val = y.subs({
            t: t_val,
            k: k_val,
            T: T_val,
            s_k: s_k_val_y,
            v_k: v_k_val_y,
            v_k_next: v_k_next_val_y
        }).evalf()
    
    y_vals.append(float(y_val))
# print(x_vals)
# print(y_vals)
plt.figure(figsize=(12, 6))
# plt.plot(t_vals, x_vals, color='blue', label='Nonlinear interpolator')
plt.plot(x_vals, y_vals, color='blue', label='Nonlinear interpolator')
# plt.plot([i * T_val for i in range(l)], s_vals, color='green', label='Linear interpolator')
plt.plot(s_vals_x, s_vals_y, color='green', label='Linear interpolator')
# plt.scatter([i * T_val for i in range(l)], s_vals, color='red', label='Waypoints')
plt.xlabel('Time')
plt.ylabel('Position')
plt.title('Interpolated Trajectory vs Linear Waypoints')
plt.legend()
plt.grid(True)
plt.show()