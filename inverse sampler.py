import matplotlib.pyplot as plt
import numpy as np

# Discrete time signal values
x_d = np.array([1, 2, 1.5, 3, 2.5])
T = 1  # Sampling period
k = np.arange(len(x_d))  # Discrete time points

# Linear interpolation function
def linear_interpolate(x_d, T, k):
    t = np.linspace(0, (len(x_d)-1)*T, 500)  # Fine-grained time points
    x_hat = np.zeros_like(t)
    for i in range(len(k)-1):
        mask = (t >= k[i]*T) & (t <= k[i+1]*T)
        x_hat[mask] = x_d[i] + (t[mask] - k[i]*T) * (x_d[i+1] - x_d[i]) / T
    return t, x_hat

# Define epsilon and delta
epsilon = 0.3
delta = 0.2

# Get interpolated signal
t, x_hat = linear_interpolate(x_d, T, k)

# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(k*T, x_d, 'bo', label='Discrete-time signal $x_d$')
plt.plot(t, x_hat, 'r-', label='Interpolated signal $\\hat{x}(t)$')
plt.fill_between(t, x_hat - delta, x_hat + delta, color='yellow', alpha=0.3, label='$\delta$-tube')
plt.plot(k*T, x_d, 'bo')  # discrete points

# Annotate epsilon bounds
for i, xd_i in enumerate(x_d):
    plt.plot([k[i]*T, k[i]*T], [xd_i - epsilon, xd_i + epsilon], 'g--', alpha=0.7)
    plt.annotate('$\epsilon$', (k[i]*T, xd_i + epsilon), textcoords="offset points", xytext=(-15,5), ha='center')

plt.xlabel('Time $t$')
plt.ylabel('Signal Value')
plt.title('Illustration of $(\epsilon, \delta)$-T Inverse Sampler')
plt.legend()
plt.grid(True)
plt.show()
