import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# Initialization
tstart = 0
tstop = 60
increment = 0.1

# Initial condition
x_init = [0, 0, 0, 0]  # [x, y, dx/dt, dy/dt]
t = np.arange(tstart, tstop+1, increment)

# Function that returns dx/dt and dy/dt
def mydiff(x, t):
    c1 = 4  # Damping constant
    c2 = 3  # Damping constant
    k = 2  # Stiffness of the spring
    m = 20  # Mass
    Fx = 5  # Force along x-axis
    Fy = 3  # Force along y-axis

    dxdt = x[2] #vel 1
    dydt = x[3] #vel 2
    dvxdt = (Fx - c1*x[2] - k*x[0])/m
    dvydt = (Fy - c2*x[3] - k*x[1])/m

    return [dxdt, dydt, dvxdt, dvydt]

# Solve ODE
x = odeint(mydiff, x_init, t)
x1 = x[:, 0]  # Posizione lungo x
x2 = x[:, 2]  # Velocità lungo x
y1 = x[:, 1]  # Posizione lungo y
y2 = x[:, 3]  # Velocità lungo y

# Plot the Results
plt.plot(t, x1, label='x_pos')
plt.plot(t, y1, label='y_pos')
plt.plot(t, x2, label='x_vel')
plt.plot(t, y2, label='y_vel')
plt.title('Simulation of 2D Mass-Spring-Damper System')
plt.xlabel('t')
plt.ylabel('Position')
plt.legend()
plt.grid()
plt.show()
