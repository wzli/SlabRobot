#!/usr/bin/env python3

import sympy as sp
from sympy.matrices import Matrix

# params
l_c, l_b, l_f, m_c, m_b, m_bl, m_bw, m_f, m_fl, m_fw, g = sp.symbols(
    "l_c l_b l_f m_c m_b m_bl m_bw m_f m_fl m_fw g", real=True, positive=True
)

# input vector
u_x, u_b, u_f = sp.symbols("u_x u_b u_f", real=True)
u = Matrix([u_x, 0, 0, u_b, u_f])

# state vector
x, y, z_c, z_b, z_f = sp.symbols("x y z_c z_b z_f", real=True)
q = Matrix([x, y, z_c, z_b, z_f])

# state derivative vector
dx, dy, dz_c, dz_b, dz_f = sp.symbols("dx dy dz_c dz_b dz_f", real=True)
dq = Matrix([dx, dy, dz_c, dz_b, dz_f])

# state double derivative vector
ddx, ddy, ddz_c, ddz_b, ddz_f = sp.symbols("ddx ddy ddz_c ddz_b ddz_f", real=True)
ddq = Matrix([ddx, ddy, ddz_c, ddz_b, ddz_f])


def rot(a):
    return Matrix([[sp.cos(a), -sp.sin(a)], [sp.sin(a), sp.cos(a)]])


# position
p_bw = Matrix([x, y])
p_b = p_bw + rot(z_c + z_b) * Matrix([l_b, 0])
p_f = p_b + rot(z_c) * Matrix([l_c, 0])
p_fw = p_f + rot(z_c + z_f) * Matrix([l_f, 0])
p_bl = (p_b + p_bw) / 2
p_fl = (p_f + p_fw) / 2
p_c = (p_b + p_f) / 2

# velocity
v_c = p_c.jacobian(q) * dq
v_b = p_b.jacobian(q) * dq
v_bl = p_bl.jacobian(q) * dq
v_bw = p_bw.jacobian(q) * dq
v_f = p_f.jacobian(q) * dq
v_fl = p_fl.jacobian(q) * dq
v_fw = p_fw.jacobian(q) * dq

# kinematic energy
T_c = m_c * v_c.T * v_c / 2
T_b = m_b * v_b.T * v_b / 2
T_bl = m_bl * v_bl.T * v_bl / 2
T_bw = m_bw * v_bw.T * v_bw / 2
T_f = m_f * v_f.T * v_f / 2
T_fl = m_fl * v_fl.T * v_fl / 2
T_fw = m_fw * v_fw.T * v_fw / 2

# potential energy
V_c = Matrix([g * m_c * p_c[1]])
V_b = Matrix([g * m_b * p_b[1]])
V_bl = Matrix([g * m_bl * p_bl[1]])
V_bw = Matrix([g * m_bw * p_bw[1]])
V_f = Matrix([g * m_f * p_f[1]])
V_fl = Matrix([g * m_fl * p_fl[1]])
V_fw = Matrix([g * m_fw * p_fw[1]])

# lagrangian
T = T_c + T_b + T_bl + T_bw + T_f + T_fl + T_fw
V = V_c + V_b + V_bl + V_bw + V_f + V_fl + V_fw
L = T - V

# euler-lagrange equation
dL_by_dq = L.jacobian(q).T
dL_by_ddq = L.jacobian(dq)
by_dt = (dL_by_ddq.jacobian(q) * dq) + (dL_by_ddq.jacobian(dq) * ddq)
euler_lagrange = dL_by_dq - by_dt + u

# solve the lagrange equation for qddot and simplify
print("Calculations take a while...")
euler_lagrange.simplify()
sol = sp.solvers.solve(euler_lagrange, ddq)

for e in sol:
    e.simplify()

print(sol)

"""
subs = {
    l_c: 0.4,
    l_b: 0.35,
    l_f: 0.35,
    x: 0,
    y: 0,
    z_c: 25 * sp.pi / 180,
    z_b: 33 * sp.pi / 180,
    z_f: 30 * sp.pi / 180,
}
print(p_b)
print(p_b.evalf(subs=subs))
print(p_f.evalf(subs=subs))
print(p_fw.evalf(subs=subs))
"""
