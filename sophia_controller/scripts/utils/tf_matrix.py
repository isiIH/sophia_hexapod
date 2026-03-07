import numpy as np

def T(x, y, z, roll, pitch, yaw):
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)

    return np.array([
        [cp*cy,             -cp*sy,             sp,     x],
        [sr*sp*cy + cr*sy,  -sr*sp*sy + cr*cy,  -sr*cp, y],
        [-cr*sp*cy + sr*sy, cr*sp*sy + sr*cy,   cr*cp,  z],
        [0,                 0,                  0,      1]
    ])