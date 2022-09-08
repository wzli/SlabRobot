from casadi import *

# variables

p = SX.sym("p", 4)

g = p[0]
l_b = p[1]
m_b = p[2]
m_bw = p[3]

q = SX.sym("q", 2)
qd = SX.sym("qd", 2)
qdd = SX.sym("qdd", 2)

x = q[0]
a = q[1]

u = vertcat(SX.sym("u_x"), 0)


def rot(v):
    return horzcat(vertcat(cos(v), sin(v)), vertcat(sin(v), -cos(v)))


# positions

p_bw = vertcat(x, 0)
p_b = p_bw + rot(a) @ vertcat(l_b, 0)

# velocities
v_b = jtimes(p_b, q, qd)
v_bw = jtimes(p_bw, q, qd)

# energies

T_b = m_b * (v_b.T @ v_b) / 2
T_bw = m_bw * (v_bw.T @ v_bw) / 2
T = T_b + T_bw

V_b = g * m_b * p_b[1]
V_bw = g * m_bw * p_bw[1]
V = V_b + V_bw

# euler-lagrange equation
L = T - V
partial_L_by_qd = jacobian(L, q)
partial_L_by_qdd = jacobian(L, qd)
by_dt = jtimes(partial_L_by_qdd, q, qd) + jtimes(partial_L_by_qdd, qd, qdd)

euler_lagrange = partial_L_by_qd.T - by_dt.T + u

dae_x = vertcat(q, qd)
dae_p = vertcat(p, u[0])

# Differential equation
f_x = vertcat(qd, qdd)

# Algebraic equation
f_z = euler_lagrange

# Lagrange cost term (quadrature)
# f_q = u[0]**2 - (10**2) * sin(a)
f_q = -sin(a) + v_b[0]**2 / 100 #x**2 / 25

# Create an integrator
dae = {"x": dae_x, "z": qdd, "p": dae_p, "ode": f_x, "alg": f_z, "quad": f_q}
step_size = 1 / 50
opts = {"tf": step_size}  # interval length
I = integrator("I", "idas", dae, opts)

"""
x0 = [0, 0, 0, 0]
z0 = [0, 0]
p0 = [9.8, 1, 1, 10, -1]
r = I(x0=x0, z0=z0, p=p0)

out = []
for i in range(1000):
    r = I(x0=x0, z0=z0, p=p0)
    out.append(r["xf"])
    x0 = r["xf"]
    z0 = r["zf"]
"""

# Number of intervals
nk = 100

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
solver = nlpsol("solver", "ipopt", nlp)  # , opts)

# Pass bounds, initial guess and solve NLP
sol = solver(
    lbx=lbw,  # Lower variable bound
    ubx=ubw,  # Upper variable bound
    p=[9.8, 1, 1, 1] + [0, -pi / 2, 0, 0] + [0, 0],
    lbg=0.0,  # Lower constraint bound
    ubg=0.0,  # Upper constraint bound
    x0=0.0,
)  # Initial guess

# Plot the results
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig, ax = plt.subplots()

ax.set_xlim([-10, 10])
ax.set_ylim([-2, 2])
ax.set_aspect(1)
(line,) = ax.plot([0], [0], marker="o")


def animate(i):
    i = (i % nk) * 5
    x = sol["x"][i + 1]
    c = np.cos(sol["x"][i + 2])
    s = np.sin(sol["x"][i + 2])
    line.set_data(np.array([x, x + c]), np.array([0, s]))  # update the data.
    return (line,)


ani = animation.FuncAnimation(fig, animate, interval=step_size * 1000, blit=True)

plt.grid()
plt.show()
