import numpy as np
from scipy.optimize import fsolve






def solve_thetas(Zp, Yp, Xp, A, B, C, T):
    if Yp < 0.0005:
        Yp = 0.0005        
        
    theta_a = np.atan2((Zp-A),Yp)

    ca = np.cos(theta_a)    
    S = Yp/ca - B
    R = np.hypot(Xp, S)           # sqrt(Xp^2 + S^2)
    phi = np.arctan2(S, Xp)
    K = (R*R + C*C - T*T) / (2.0*C)

    if abs(K/R) > 1.0 + 1e-12:
        return []  # no real solutions

    # clamp for numeric stability
    val = np.clip(K/R, -1.0, 1.0)
    acos_val = np.arccos(val)

    thetab_solutions = [phi + acos_val, phi - acos_val]
    solutions = []
    for tb in thetab_solutions:
        cb = np.cos(tb); sb = np.sin(tb)
        cd = (Xp - C*cb) / T
        sd = (S  - C*sb) / T
        # numeric clamp
        cd = np.clip(cd, -1.0, 1.0)
        sd = np.clip(sd, -1.0, 1.0)
        td = np.arctan2(sd, cd)
        solutions.append((theta_a, tb, td))
    return solutions

Xp = 0
Yp = 10
Zp = 25

A = 10
B = 10
C = 8
T = 8

theta_a, theta_b, theta_d  = solve_thetas(Zp, Yp, Xp, A, B, C, T)[0]
theta_c = theta_b - theta_d

print('theta_a = %.2f' %  np.rad2deg(theta_a))
print('theta_b = %.2f' %  np.rad2deg(theta_b))
print('theta_c = %.2f' %  np.rad2deg(theta_c))