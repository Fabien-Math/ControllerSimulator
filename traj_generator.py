import numpy as np

N = 50

T = np.linspace(0, 2*np.pi, N)

X = np.pow(T, 2)
Y = np.sin(T)*2
Z = -10 * T + np.cos(T)


P = np.zeros_like(T)
Q = np.zeros_like(T)
R = np.linspace(-np.pi, np.pi, N)


for i in range(N):
    print(f"- [{X[i]:.3f}, {Y[i]:.3f}, {Z[i]:.3f}, {P[i]:.3f}, {Q[i]:.3f}, {R[i]:.3f}]")