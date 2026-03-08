import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 1. Define the points
points = np.array([
    [279.488, -19.312, -538.991],
    [301.416, -19.7475, -539.054],
    [324.189, -19.8434, -539.222],
    [348.793, -20.0385, -539.401]
])

# 2. Extract X, Y, Z for plotting
x, y, z = points[:, 0], points[:, 1], points[:, 2]

# 3. Calculate the Line of Best Fit using SVD
# The line passes through the centroid (mean) of the points
mean = np.mean(points, axis=0)
centered_points = points - mean

# SVD finds the principal component (direction of the line)
u, s, vh = np.linalg.svd(centered_points)
direction = vh[0, :] # The first singular vector

# Generate points along this line for plotting
# Adjust the range (-50, 50) to extend the line as needed
t = np.linspace(-50, 50, 100)
line = mean + t[:, np.newaxis] * direction

# 4. Create the 3D Plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Plot original points
ax.scatter(x, y, z, color='red', s=100, label='Data Points')

# Plot the line of best fit
ax.plot(line[:, 0], line[:, 1], line[:, 2], color='blue', linewidth=2, label='Line of Best Fit')

# Labeling
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.set_title('3D Points with Line of Best Fit')
ax.legend()

# Label individual points
for i, txt in enumerate(['P1', 'P2', 'P3', 'P4']):
    ax.text(x[i], y[i], z[i], f' {txt}', size=10, zorder=1, color='black')

mean = np.mean(points, axis=0)
centered_points = points - mean
u, s, vh = np.linalg.svd(centered_points)
direction = vh[0, :] # Unit vector of the line

# 3. Calculate distance from each point to the line
# Distance = || (P - A) - ((P - A) · d) * d ||
distances = []
for p in points:
    pa = p - mean
    proj = np.dot(pa, direction) * direction
    dist = np.linalg.norm(pa - proj)
    distances.append(dist)

# 4. Final Stats
print(f"Min Error:  {np.min(distances):.4f}")
print(f"Max Error:  {np.max(distances):.4f}")
print(f"Mean Error: {np.mean(distances):.4f}")

plt.show()
