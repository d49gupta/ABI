import numpy as np
import matplotlib.pyplot as plt

def analyze_line_fit(data, ideal_y=0.0, show_plot=True):
    x_actual = [pt[0] for pt in data]
    y_actual = [pt[1] for pt in data]
    x = np.array(x_actual)
    y = np.array(y_actual)

    m, b = np.polyfit(x, y, 1)
    y_fit = m * x + b

    y_ideal = np.full_like(x, ideal_y)
    errors = y_fit - y_ideal
    mean_error = np.mean(np.abs(errors))
    max_error = np.max(errors)
    min_error = np.min(errors)

    print("Mean Error:", mean_error)
    print("Max Error:", max_error)
    print("Min Error:", min_error)

    if show_plot:
        plt.figure(figsize=(7, 5))
        plt.scatter(x, y, color='red', label='Measured Points')
        plt.plot(x, y_fit, 'b--', label=f'Best Fit: y={m:.4f}x+{b:.4f}')
        plt.plot(x, y_ideal, 'k-', label=f'Ideal Line: y={ideal_y}')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.title('Best-Fit vs Ideal Line')
        plt.grid(True)
        plt.show()

    return

def degree_error(points):
    P0 = np.array(points[0])
    P1 = np.array(points[1])
    P2 = np.array(points[2])

    v1 = P1 - P0
    v2 = P2 - P0

    dot = np.dot(v1, v2)
    norm = np.linalg.norm(v1) * np.linalg.norm(v2)

    cos_theta = dot / norm
    cos_theta = np.clip(cos_theta, -1.0, 1.0)

    theta = np.arccos(cos_theta) * 180 / 3.14159
    return theta