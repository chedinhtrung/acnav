import pyvista as pv
import numpy as np
import quaternion
import time

# Create the plotter
plotter = pv.Plotter()
plotter.add_axes()
plotter.show(auto_close=False)  # Keep the window open

# Create a coordinate triad
axes = pv.Arrow(start=[0, 0, 0], direction=[1, 0, 0], scale="auto")
actor = plotter.add_mesh(axes, color='red', name='xaxis')

# Add Y and Z manually
y_actor = plotter.add_mesh(pv.Arrow([0, 0, 0], [0, 1, 0]), color='green', name='yaxis')
z_actor = plotter.add_mesh(pv.Arrow([0, 0, 0], [0, 0, 1]), color='blue', name='zaxis')

# Start plotting in a separate thread (non-blocking)
plotter.show(auto_close=False, interactive_update=True)

# Live update loop
theta = 0
while True:
    # Example: Rotate around Z
    theta += 0.001
    q = quaternion.from_rotation_vector([0, 0, theta])  # z-axis rotation

    # Convert quaternion to rotation matrix
    R = quaternion.as_rotation_matrix(q)

    # Update the coordinate axes
    plotter.update_coordinates(pv.Arrow([0, 0, 0], R[:, 0]), render=True, name='xaxis')
    plotter.update_coordinates(pv.Arrow([0, 0, 0], R[:, 1]), render=True, name='yaxis')
    plotter.update_coordinates(pv.Arrow([0, 0, 0], R[:, 2]), render=True, name='zaxis')

    plotter.render()
    #time.sleep(0.02)  # ~50 Hz update
