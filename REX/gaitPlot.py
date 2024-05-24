import numpy as np
import matplotlib.pyplot as plt

from gait import walkGait

gait_planner = walkGait()
step_offset = np.array([0.5, 0, 0, 0.5])
initial_foot_positions = np.array(
    [
        [0.1, 0, 0.2],
        [0.1, 0, 0.2],
        [-0.1, 0, 0.2],
        [-0.1, 0, 0.2],
    ]
)
trajectories = gait_planner.simulateCycle(
    0.1, 0, 0, step_offset, initial_foot_positions, steps=100
)

# Plotting
fig, axs = plt.subplots(2, 2, figsize=(12, 10), subplot_kw={"projection": "3d"})

# Store the last scatter plot handle for the colorbar reference
sc_handle = None

for i, ax in enumerate(axs.flat):
    ax.plot(
        trajectories[i]["x"],
        trajectories[i]["y"],
        trajectories[i]["z"],
    )
    sc = ax.scatter(
        trajectories[i]["x"],
        trajectories[i]["y"],
        trajectories[i]["z"],
        c=np.linspace(0, 1, len(trajectories[i]["x"])),
    )
    sc_handle = sc  # Update with the last scatter handle
    ax.set_xlabel("X position")
    ax.set_ylabel("Y position")
    ax.set_zlabel("Z position")
    ax.set_title(f"Foot {i+1} Cycle Trajectory")

# Create a colorbar with a bit of padding from the subplots
fig.subplots_adjust(right=1.6)  # Adjust the right margin
cbar_ax = fig.add_axes([0.85, 0.15, 0.01, 0.7])  # Add axes for the colorbar
cbar = fig.colorbar(sc_handle, cax=cbar_ax)
cbar.set_label("Duty Cycle Time progression")

plt.tight_layout()
plt.show()
