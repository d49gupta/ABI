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

# --- SAMPLED POINTS 
# PERS robtarget calibration_pose{4} := 
# [
# [[344.783,-17.973,-539.875],[0.00246905,0.965822,0.259055,0.00849908],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,0]],
# [[366.811,-18.3668,-540.218],[0.0032326,0.965652,0.259559,0.0116495],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,25.4772]],
# [[389.258,-18.5014,-540.432],[0.00400952,0.965462,0.260091,0.014847],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,50.8005]],
# [[412.395,-18.5168,-540.671],[0.00474478,0.965308,0.260472,0.0176558],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,76.5856]]
# ];



# PERS robtarget Point1 := [[492.295,-5.70785,-862.431],[0.000311486,0.965891,0.258945,0.00109665],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1467.83]];
# PERS robtarget Point2 := [[440.7,-6.10673,-862.009],[0.000117693,-0.965899,-0.258919,-0.000346284],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1467.83]];
# PERS robtarget Point3 := [[493.242,-57.3789,-862.489],[6.05152E-05,-0.965917,-0.258845,-0.00168306],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,1467.83]];