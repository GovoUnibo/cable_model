import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

def grav(x, t):
    g = -9.81
    dxdt = x[1] # velocità
    dvxdt = g
    return [dxdt, dvxdt]

x_init = [0, 0]
t = np.arange(0, 1, 0.01)
x = odeint(grav, x_init, t)

# Grafico della posizione nel tempo
plt.figure()
plt.plot(t, x[:, 0])
plt.title('Posizione nel tempo')
plt.xlabel('Tempo')
plt.ylabel('Posizione')
plt.grid(True)

# Grafico della velocità nel tempo
plt.figure()
plt.plot(t, x[:, 1])
plt.title('Velocità nel tempo')
plt.xlabel('Tempo')
plt.ylabel('Velocità')
plt.grid(True)

plt.show()
