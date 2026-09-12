import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


def ellipse(ax, semi_major, semi_minor, tilt_deg, color):
    """Draw the ring swept by the roll axis as an ellipse (a circle seen edge-on)."""
    t = np.linspace(0, 2 * np.pi, 400)
    x = semi_major * np.cos(t)
    y = semi_minor * np.sin(t)
    a = np.radians(tilt_deg)
    xr = x * np.cos(a) - y * np.sin(a)
    yr = x * np.sin(a) + y * np.cos(a)
    ax.plot(xr, yr, color=color, lw=2)


def gimbal(ax, pitch_deg, title):
    # Outer sphere / yaw frame
    ax.add_patch(Circle((0, 0), 1.0, fill=False, ec="tab:blue", lw=2))
    ax.text(0.02, 1.08, r"$z$", color="tab:blue", fontsize=13, ha="center")

    # Roll ring: flattens out and lines up with the x arrow as pitch -> 90 deg
    tilt = 90.0 - pitch_deg
    ellipse(ax, 1.0, 0.45 * np.cos(np.radians(pitch_deg)) + 0.05, tilt, "tab:purple")
    ax.text(-1.12, 0.03, r"$y$", color="tab:purple", fontsize=13, va="center")

    # x axis after the pitch rotation
    a = np.radians(tilt)
    ax.annotate("", xy=(np.cos(a), np.sin(a)), xytext=(-np.cos(a), -np.sin(a)),
                arrowprops=dict(arrowstyle="-|>", color="tab:orange", lw=2))
    ax.text(np.cos(a) + 0.12, np.sin(a) + 0.08, r"$x$", color="tab:orange", fontsize=13)

    ax.set_title(title, fontsize=14)
    ax.set_xlim(-1.45, 1.45)
    ax.set_ylim(-1.3, 1.3)
    ax.set_aspect("equal")
    ax.axis("off")


fig, axes = plt.subplots(1, 2, figsize=(9, 4.5))
gimbal(axes[0], 35.0, r"$\theta \neq 90^\circ$")
gimbal(axes[1], 90.0, r"$\theta = 90^\circ$")
fig.tight_layout()
fig.savefig("../Latex/Figures/Problem1D.png", dpi=300, bbox_inches="tight")
