import numpy as np
from numpy.linalg import norm



# Given rotation matrix
R = np.array([
    [ 0.6533,  0.6403,  0.4040],
    [-0.3488,  0.7282, -0.5900],
    [-0.6719,  0.2445,  0.6991]
])

r13, r23, r33 = R[0, 2], R[1, 2], R[2, 2]
r31, r32 = R[2, 0], R[2, 1]
r11, r22 = R[0 , 0], R[1 , 1]
r21 , r12 = R[1, 0] , R[0, 1]


# ZYZ Euler angles: R = Rz(phi) * Ry(theta) * Rz(psi)
theta = np.arctan2(np.sqrt(r13**2 + r23**2), r33)
phi = np.arctan2(r23, r13)
psi = np.arctan2(r32, -r31)

# Angle-Axis
diffVector = np.array([
    [r32 - r23],
    [r13 - r31],
    [r21 - r12]
])

theta2 = np.arctan2( .5 * norm(diffVector) , (np.trace(R) - 1)/2)
k = 1/ (2 * np.sin(theta2))  * diffVector

# Quaternion

q_0 = np.sqrt((1 + np.trace(R))/4)
q_1 = (r32 - r23) / (4 * q_0)
q_2 = (r13 - r31) / (4 * q_0)
q_3 = (r21 - r12) / (4 * q_0)

print(f"phi   = {np.degrees(phi):.4f} deg")
print(f"theta = {np.degrees(theta):.4f} deg")
print(f"psi   = {np.degrees(psi):.4f} deg")
print(f"theta2 = {np.degrees(theta2):.4f} deg")
print(f"k = {k.flatten()}")
print(f"q = [{q_0:.4f}, {q_1:.4f}, {q_2:.4f}, {q_3:.4f}]")
