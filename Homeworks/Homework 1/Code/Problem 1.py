import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FIGURES_DIR = os.path.join(SCRIPT_DIR, '..', 'Latex', 'Figures')
FIGURE_PATH = os.path.join(FIGURES_DIR, 'Problem1A.png')


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
        xytext = (18 * np.cos(t), 18 * np.sin(t))
        ax.annotate(label, origin, textcoords='offset points', xytext=xytext, fontsize=11,
                    fontweight='bold', color=color, ha='center', va='center')


steps = [
    [('translate', 1.0), ('rotate', -45)],
    [('translate', 0.5), ('rotate', 30)],
    [('translate', -2.0)],
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
            pos = pos + val * rot(phi)[:, 1]
            path.append(pos.copy())
    frames.append((pos.copy(), phi))

path = np.array(path)
final_pos, final_phi = frames[-1]
obj_pos = final_pos + 1.5 * rot(final_phi)[:, 0]

colors = plt.get_cmap('tab10').colors

fig, ax = plt.subplots(figsize=(8, 8))
ax.plot(path[:, 0], path[:, 1], 'k-o', linewidth=2, markersize=6, zorder=2)
ax.plot(*obj_pos, 'b^', markersize=10, zorder=3)

for i, (p, ang) in enumerate(frames):
    draw_frame(ax, p, ang, colors[i % len(colors)], label=str(i), label_angle=-90 + 65 * i)

obj_color = colors[len(frames) % len(colors)]
draw_frame(ax, obj_pos, final_phi, obj_color, label='obj', label_angle=-90)

all_pts = np.vstack([path, obj_pos])
xmin, ymin = np.floor(all_pts.min(axis=0) * 2 - 1) / 2
xmax, ymax = np.ceil(all_pts.max(axis=0) * 2 + 1) / 2

ax.set_xticks(np.arange(xmin, xmax + 0.01, 0.5))
ax.set_yticks(np.arange(ymin, ymax + 0.01, 0.5))
ax.set_xlim(xmin, xmax)
ax.set_ylim(ymin, ymax)
ax.set_aspect('equal')
ax.grid(True)
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_title('Mobile Robot Path')

legend_elements = [
    Line2D([0], [0], color='k', marker='o', label='Robot path'),
    Line2D([0], [0], color='b', marker='^', linestyle='None', label='Object'),
    Line2D([0], [0], color='gray', linestyle='solid', label='x axis'),
    Line2D([0], [0], color='gray', linestyle='dashed', label='y axis'),
]
ax.legend(handles=legend_elements, loc='upper right')

os.makedirs(FIGURES_DIR, exist_ok=True)
plt.savefig(FIGURE_PATH, dpi=200)
plt.show()
