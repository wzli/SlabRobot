#!/usr/bin/env python3

from casadi import *


def rot(v):
    return horzcat(vertcat(cos(v), sin(v)), vertcat(sin(v), -cos(v)))


# Number of intervals
nk = 1000
params = [9.8, 0.5, 0.5, 0.5, 5, 5, 5, 5]


# define parameters
p = SX.sym("p", 8)
g = p[0]  # gravity
l_c = p[1]  # length of body
l_b = p[2]  # length of back legs
l_f = p[3]  # length of front legs
m_b = p[4]  # mass of back joints
m_f = p[5]  # mass of front joints
m_bw = p[6]  # mass of back wheels
m_fw = p[7]  # mass of front wheels


# define control inputs
u = SX.sym("u", 3)
u_x = u[0]  # velocity of ground wheel
u_b = u[1]  # angular velocity of back joint
u_f = u[2]  # angular velocity of front joint

# define joint states
q = SX.sym("q", 4)
x_bw = q[0]  # position of ground wheel
z_b = q[1]  # angle of back joint
z_f = q[2]  # angle of front joint
z_c = q[3]  # angle of body (global)

# define joint state derivatives
zd_c = SX.sym("zd_c")  # angular velocity of body (global)
zdd_c = SX.sym("zdd_c")  # angular acceleration of body (global)
qd = vertcat(u_x, u_b, u_f, zd_c)

# positions
p_bw = vertcat(x_bw, 0)  # position of back wheel
p_b = p_bw + rot(z_c + z_b) @ vertcat(l_b, 0)  # position of back joint
p_f = p_b + rot(z_c) @ vertcat(l_c, 0)  # position of front joint
p_fw = p_f + rot(z_c + z_f) @ vertcat(l_f, 0)  # position of front wheel

forward_kinematics = Function("forward_kinematics", [q, p], [p_bw, p_b, p_f, p_fw])

# velocities
v_b = jtimes(p_b, q, qd)  # velocity of back joint
v_f = jtimes(p_f, q, qd)  # velocity of front joint
v_bw = jtimes(p_bw, q, qd)  # velocity of back wheel
v_fw = jtimes(p_fw, q, qd)  # velocity of back wheel

# kinetic energies
T_b = m_b * (v_b.T @ v_b) / 2  # KE of back joint
T_f = m_f * (v_f.T @ v_f) / 2  # KE of front joint
T_bw = m_bw * (v_bw.T @ v_bw) / 2  # KE of back wheel
T_fw = m_fw * (v_fw.T @ v_fw) / 2  # KE of front wheel
T = T_b + T_f + T_bw + T_fw

# potential energies
V_b = g * m_b * p_b[1]  # PE of back joint
V_f = g * m_f * p_f[1]  # PE of front joint
V_bw = g * m_bw * p_bw[1]  # PE of back wheel
V_fw = g * m_fw * p_fw[1]  # PE of front wheel
V = V_b + V_f + V_bw + V_fw

# Euler-Lagrange equation
L = T - V
partial_L_by_qd = jacobian(L, z_c)
partial_L_by_qdd = jacobian(L, zd_c)
by_dt = jtimes(partial_L_by_qdd, z_c, zd_c) + jtimes(partial_L_by_qdd, zd_c, zdd_c)
euler_lagrange = partial_L_by_qd.T - by_dt.T

# solve DAE
dae_x = vertcat(q, zd_c)  # States
dae_p = vertcat(p, u)  # Params
f_x = vertcat(qd, zdd_c)  # Differential equation
f_z = euler_lagrange  # Algebraic equation
f_q = 0  # Lagrange cost term (quadrature)


# Create an integrator
dae = {"x": dae_x, "z": zdd_c, "p": dae_p, "ode": f_x, "alg": f_z, "quad": f_q}
step_size = 1 / 100
opts = {"tf": step_size}  # interval length
I = integrator("I", "idas", dae, opts)

x0 = [0, 0, 0, 0.01 + pi / 2, 0]
z0 = 0
p0 = params + [0, 0.5, 0]
r = I(x0=x0, z0=z0, p=p0)

out = []
for i in range(nk):
    r = I(x0=x0, z0=z0, p=p0)
    out.append(r["xf"])
    x0 = r["xf"]
    z0 = r["zf"]

"""


# Start with an empty NLP
w = []  # List of variables
lbw = []  # Lower bounds on w
ubw = []  # Upper bounds on w
G = []  # Constraints
J = 0  # Cost function


# Initial conditions
P = MX.sym("P", 4)
X0 = MX.sym("X0", 4)
Z0 = MX.sym("Z0", 2)

Xk = X0
Zk = Z0

# Loop over all intervals
for k in range(nk):
    # Local control
    Uk = MX.sym("U" + str(k))
    w.append(Uk)
    lbw += [-50]
    ubw += [50]

    # Call integrator function
    Ik = I(x0=Xk, z0=Zk, p=vertcat(P, Uk))
    Xk = Ik["xf"]
    Zk = Ik["zf"]
    J = J + Ik["qf"]  # Sum quadratures

    # "Lift" the variable
    X_prev = Xk
    Xk = MX.sym("X" + str(k + 1), 4)
    w.append(Xk)
    lbw += [-inf] * 4
    ubw += [inf] * 4
    G.append(X_prev - Xk)


# Allocate an NLP solver
nlp = {"x": vertcat(*w), "f": J, "g": vertcat(*G), "p": vertcat(P, X0, Z0)}
# opts = {'ipopt.linear_solver':'ma27'}
solver = nlpsol("solver", "ipopt", nlp)# , opts)

# Pass bounds, initial guess and solve NLP
sol = solver(
    lbx=lbw,  # Lower variable bound
    ubx=ubw,  # Upper variable bound
    p=[9.8, 1, 1, 1] + [0, -pi / 2, 0, 0] + [0, 0],
    lbg=0.0,  # Lower constraint bound
    ubg=0.0,  # Upper constraint bound
    x0=0.0,
)  # Initial guess
"""

# Plot the results
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig, ax = plt.subplots()

ax.set_xlim([-10, 10])
ax.set_ylim([-2, 2])
ax.set_aspect(1)
(line,) = ax.plot([0], [0], marker="o")


def animate(i):
    """
    i = (i % nk) * 5
    x = sol["x"][i + 1]
    c = np.cos(sol["x"][i + 2])
    s = np.sin(sol["x"][i + 2])
    line.set_data(np.array([x, x + c]), np.array([0, s]))  # update the data.
    """
    i = i % nk
    points = forward_kinematics(out[i][:-1], params)
    xs = np.array([p[0] for p in points])
    ys = np.array([p[1] for p in points])
    line.set_data(xs, ys)  # update the data.
    return (line,)


ani = animation.FuncAnimation(fig, animate, interval=step_size * 1000, blit=True)

plt.grid()
plt.show()
