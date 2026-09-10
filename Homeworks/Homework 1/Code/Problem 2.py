import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FIGURES_DIR = os.path.join(SCRIPT_DIR, '..', 'Latex', 'Figures')
FIGURE_PATH = os.path.join(FIGURES_DIR, 'Problem2A.png')


def rot(deg):
    t = np.radians(deg)
    return np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])


def draw_frame(ax, origin, phi, color, length=0.3, label=None, label_angle=0.0):
    R = rot(phi)
    x_axis = R[:, 0] * length
    y_axis = R[:, 1] * length
    x_patch = ax.arrow(*origin, *x_axis, head_width=0.05, color=color, length_includes_head=True, zorder=3)
    x_patch.set_linestyle('solid')
    y_patch = ax.arrow(*origin, *y_axis, head_width=0.05, color=color, length_includes_head=True, zorder=3)
    y_patch.set_linestyle('dashed')
    if label:
        t = np.radians(label_angle)
        xytext = (26 * np.cos(t), 26 * np.sin(t))
        ax.annotate(label, origin, textcoords='offset points', xytext=xytext, fontsize=11,
                    fontweight='bold', color=color, ha='center', va='center')


theta0 = 45.0   # deg, joint 0 (rotary)
a1 = 0.5        # m, joint 1 (prismatic)
theta2 = 45.0   # deg, joint 2 (rotary)

# Each step is (fixed/variable rotation about z, then translation along the resulting x axis)
steps = [
    [('rotate', theta0), ('translate', 1.0)],   # joint 0 + link 1
    [('rotate', -45.0), ('translate', a1)],     # fixed -45 offset + prismatic link 2
    [('rotate', theta2), ('translate', 1.0)],   # joint 2 + link 3
]

pos = np.array([0.0, 0.0])
phi = 0.0
path = [pos.copy()]
frames = [(pos.copy(), phi)]

for step in steps:
    for kind, val in step:
        if kind == 'rotate':
            phi += val
        else:
            pos = pos + val * rot(phi)[:, 0]
            path.append(pos.copy())
    frames.append((pos.copy(), phi))

path = np.array(path)

colors = plt.get_cmap('tab10').colors

fig, ax = plt.subplots(figsize=(8, 8))
ax.plot(path[:, 0], path[:, 1], 'k-', linewidth=2, zorder=2)
ax.plot(*path[0], 'ko', markersize=8, zorder=3)           # O0 (rotary base)
ax.plot(*path[1], 'ks', markersize=8, zorder=3)           # O1 (prismatic joint)
ax.plot(*path[2], 'ko', markersize=8, zorder=3)           # O2 (rotary joint)
ax.plot(*path[3], 'k^', markersize=10, zorder=3)          # O3 (end effector)

frame_labels = ['0', '1', '2', '3']
for i, (p, ang) in enumerate(frames):
    draw_frame(ax, p, ang, colors[i % len(colors)], label=frame_labels[i], label_angle=ang + 225)

xmin, ymin = np.floor(path.min(axis=0) * 2 - 1) / 2
xmax, ymax = np.ceil(path.max(axis=0) * 2 + 1) / 2

ax.set_xticks(np.arange(xmin, xmax + 0.01, 0.5))
ax.set_yticks(np.arange(ymin, ymax + 0.01, 0.5))
ax.set_xlim(xmin, xmax)
ax.set_ylim(ymin, ymax)
ax.set_aspect('equal')
ax.grid(True)
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_title(r'Three-Link RPR Manipulator ($\theta_0=\pi/4$, $a_1=0.5$m, $\theta_2=\pi/4$)')

legend_elements = [
    Line2D([0], [0], color='k', marker='o', linestyle='None', label='Rotary joint'),
    Line2D([0], [0], color='k', marker='s', linestyle='None', label='Prismatic joint'),
    Line2D([0], [0], color='k', marker='^', linestyle='None', label='End effector'),
    Line2D([0], [0], color='gray', linestyle='solid', label='x axis'),
    Line2D([0], [0], color='gray', linestyle='dashed', label='y axis'),
]
ax.legend(handles=legend_elements, loc='upper left')

os.makedirs(FIGURES_DIR, exist_ok=True)
plt.savefig(FIGURE_PATH, dpi=200)
plt.show()
