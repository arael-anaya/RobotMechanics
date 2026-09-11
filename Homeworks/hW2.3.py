import numpy as np

def quat_to_R_and_axis(q0, q1, q2, q3):
    R = np.array([
        [1 - 2*(q2**2 + q3**2),     2*(q1*q2 - q0*q3),     2*(q1*q3 + q0*q2)],
        [    2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2),     2*(q2*q3 - q0*q1)],
        [    2*(q1*q3 - q0*q2),     2*(q2*q3 + q0*q1), 1 - 2*(q1**2 + q2**2)]
    ])

    theta = 2 * np.arccos(q0)
    s = np.sin(theta / 2)
    if s < 1e-8:
        k = np.array([0.0, 0.0, 0.0])
    else:
        k = np.array([q1, q2, q3]) / s

    return R, theta, k

quaternions = {
    "a": (1, 0, 0, 0),
    "b": (0, 0, 0, 1),
    "c": (0, 1, 0, 0),
    "d": (0.5332, 0.5928, 0.08311, 0.5978),
}

for label, q in quaternions.items():
    R, theta, k = quat_to_R_and_axis(*q)
    print(f"--- ({label}) q = {q} ---")
    print(f"R =\n{R}")
    print(f"theta = {np.degrees(theta):.4f} deg")
    print(f"k = {k}")
    print()
