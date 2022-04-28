from sympy import symbols, sin, cos, tan, Matrix
import sympy
import math
import numpy as np

x, y, z, phi, theta, psy, u1, u2, u3, u4 = symbols('x, y, z, phi, theta, psy, u1, u2, u3, u4')
dx, dy, dz, dphi, dtheta, dpsy = symbols('dx, dy, dz, dphi, dtheta, dpsy')

q = [x, y, z, phi, theta, psy]
u = [u1, u2, u3, u4]
dq = [dx, dy, dz, dphi, dtheta, dpsy]

ten6 = 10**-6
m, l, Ix, Iy, Iz, Ip  = 0.027, 0.046, 16.5717*ten6, 16.5717*ten6, 29.2616*ten6, 12.65625*ten6
kF, kM, w_max, w_min, g = 1.26192*10**-8, 5.9645*10**-3, 2618, 0, 9.81

t1,t2,t3 = 1/(4*kF), math.sqrt(2)/(4*kF*l), 1/(4*kM*kF)
w1 = sympy.sqrt( Matrix([t1,-t2,-t2,-t3]).dot( Matrix([u1,u2,u3,u4])) )
w2 = sympy.sqrt( Matrix([t1,-t2, t2, t3]).dot( Matrix([u1,u2,u3,u4])) )
w3 = sympy.sqrt( Matrix([t1, t2, t2,-t3]).dot( Matrix([u1,u2,u3,u4])) )
w4 = sympy.sqrt( Matrix([t1, t2,-t2, t3]).dot( Matrix([u1,u2,u3,u4])) )

Omega = w1-w2+w3-w4

# laambda = 1
# e = q - qd
# de = dq - dqd

ddx = (u1/m) * ( cos(phi)*sin(theta)*cos(psy) + sin(phi)*sin(psy) )
ddy = (u1/m) * ( cos(phi)*sin(theta)*cos(psy) - sin(phi)*sin(psy) )
ddz = (u1/m) * ( cos(phi)*cos(theta) ) - g
ddphi = dtheta*dpsy + (Iy-Iz)/Ix - Ip/Ix*Omega*dtheta + 1/Ix * u2
ddtheta = dphi*dpsy + (Iz-Ix)/Iy - Ip/Iy*Omega*dphi + 1/Iy * u3
ddpsy = dtheta*dphi + (Ix-Iy)/Iz - 1/Iz * u4



# s = de + laambda * e

