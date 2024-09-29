import sympy as sp
import numpy as np
import matplotlib.pyplot as plt
from sympy import init_printing
init_printing() 

t, k, T = sp.symbols('t k T')
s_k, s_k_next, v_k, v_k_next = sp.symbols('s_k s_k_next v_k v_k_next')

# t_p1 = t - k*T - T/2
# t_p2 = t - (k+1)*T - T/2
# # Comment below to use modified t_p2
# # t_p2 = t - (k+1)*T + T/2

# x1 = (s_k + v_k * (t - k*T) - 
#       (v_k_next - v_k) / (T**2) * t_p1**3 * (1 - 2*t_p1/T))
# x2 = (s_k_next + v_k_next * (t - (k+1)*T) +
#       (v_k_next - v_k) / T**2 * t_p2**3 * (1 + 2*t_p2/T)) 
# # Comment below to use modified x2
# # x2 = (s_k_next + v_k_next * (t - (k+1)*T) + 
# #       (v_k_next - v_k) / T**2 * t_p2**3 * (1 - 2*t_p2/T)) 

# x = sp.Piecewise(
#     (x1, sp.And(t >= (k+0.5)*T, t < (k+1)*T)),
#     (x2, sp.And(t >= (k+1)*T, t < (k+1.5)*T))
# )


# Modified implementation; doesn't start from midpoint
t_p1 = t - k*T - T/2
t_p2 = t - k*T - T/2
x1 = (s_k + v_k * (t - k*T) - 
      (v_k_next - v_k) / (T**2) * t_p1**3 * (1 + 2*t_p1/T))
x2 = (s_k + v_k * (t - k*T) -
      (v_k_next - v_k) / T**2 * t_p2**3 * (1 - 2*t_p2/T)) 
x = sp.Piecewise(
    (x1, sp.And(t >= k*T, t < (k+0.5)*T)),
    (x2, sp.And(t >= (k+0.5)*T, t < (k+1)*T))
)

n = 10
l = 5
s_vals = np.zeros(l)
v_vals = np.array([50, 0, 0, 0, 0])
for i in range(1, len(v_vals)):
    s_vals[i] = s_vals[i-1] + v_vals[i-1]
# print(s_vals)
# print(v_vals)
t_vals = np.linspace(0, l-1-1/(l*n), (l-1)*l*n)
# print(t_vals)
x_vals = []
T_val = 1

for t_val in t_vals:
    k_val = int(t_val / T_val)
    # if t_val < (k_val+0.5)*T_val:
    #     k_val -= 1
    s_k_val = s_vals[k_val]
    s_k_next_val = s_vals[k_val + 1]
    v_k_val = v_vals[k_val]
    v_k_next_val = v_vals[k_val + 1]

    x_val = x.subs({
            t: t_val,
            k: k_val,
            T: T_val,
            s_k: s_k_val,
            s_k_next: s_k_next_val,
            v_k: v_k_val,
            v_k_next: v_k_next_val
        }).evalf()
    
    x_vals.append(float(x_val))

plt.figure(figsize=(12, 6))
plt.plot(t_vals, x_vals, color='blue', label='Nonlinear interpolator')
# plt.plot([i * T_val for i in range(l)], s_vals, color='green', label='Linear interpolator')
# plt.scatter([i * T_val for i in range(l)], s_vals, color='red', label='Waypoints')
plt.xlabel('Time')
plt.ylabel('Position')
plt.title('Interpolated Trajectory vs Linear Waypoints')
plt.legend()
plt.grid(True)
plt.show()