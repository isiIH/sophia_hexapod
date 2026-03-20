import numpy as np

def T(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)

    return np.array([
        [cp*cy,             -cp*sy,             sp,     x],
        [sr*sp*cy + cr*sy,  -sr*sp*sy + cr*cy,  -sr*cp, y],
        [-cr*sp*cy + sr*sy, cr*sp*sy + sr*cy,   cr*cp,  z],
        [0,                 0,                  0,      1]
    ])