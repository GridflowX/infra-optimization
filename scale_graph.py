import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# Function to calculate total length of given edges
def total_edge_length(points, edges):
    return sum(np.linalg.norm(points[i] - points[j]) for i, j in edges)

# Dummy objective function (we only care about constraint satisfaction)
def objective(scale):
    return 0

# Constraint function: scaled total edge length must match target
def edge_length_constraint(scale, points, edges, target_total_length):
    scaled_points = points * scale
    return total_edge_length(scaled_points, edges) - target_total_length

# Adjust the polygon scaling so that the total length of given edges matches the target
def adjust_polygon_to_edge_length(points, edges, target_total_length):
    points = np.array(points)
    initial_scale = 1.0

    constraints = {
        'type': 'eq',
        'fun': lambda scale: edge_length_constraint(scale, points, edges, target_total_length)
    }

    result = minimize(
        objective,
        [initial_scale],
        method='SLSQP',
        constraints=constraints,
        bounds=[(0, None)]
    )

    optimized_scale = result.x[0]
    optimized_points = points * optimized_scale
    return optimized_points, total_edge_length(optimized_points, edges)

# Visualization function
def plot_scaled_polygon(adjusted_points, total_length, edges, stations, steiners, intermediate):
    adjusted_points = np.array(adjusted_points)

    # Index boundaries
    station_end = len(stations)
    steiner_end = station_end + len(steiners)
    intermediate_end = steiner_end + len(intermediate)

    # Extract subgroups
    station_points = adjusted_points[0:station_end]
    steiner_points = adjusted_points[station_end:steiner_end]
    intermediate_points = adjusted_points[steiner_end:intermediate_end]

    plt.figure(figsize=(8, 8))

    # Plot station points
    if len(station_points) > 0:
        plt.scatter(station_points[:, 0], station_points[:, 1], color='blue', label='Stations')
        for i, (x, y) in enumerate(station_points):
            plt.text(x, y, f"P{i}", fontsize=12, ha='right', color='blue')

    # Plot steiner points
    if len(steiner_points) > 0:
        plt.scatter(steiner_points[:, 0], steiner_points[:, 1], color='red', label='Steiner Points')
        for i, (x, y) in enumerate(steiner_points, start=station_end):
            plt.text(x, y, f"P{i}", fontsize=12, ha='right', color='red')

    # Plot intermediate points
    if len(intermediate_points) > 0:
        plt.scatter(intermediate_points[:, 0], intermediate_points[:, 1], color='teal', label='Intermediate Points')
        for i, (x, y) in enumerate(intermediate_points, start=steiner_end):
            plt.text(x, y, f"P{i}", fontsize=12, ha='right', color='teal')

    # Plot edges with lengths
    for edge in edges:
        i, j = edge
        plt.plot([adjusted_points[i, 0], adjusted_points[j, 0]],
                 [adjusted_points[i, 1], adjusted_points[j, 1]], 'k--', alpha=0.6)

        mid_x = (adjusted_points[i, 0] + adjusted_points[j, 0]) / 2
        mid_y = (adjusted_points[i, 1] + adjusted_points[j, 1]) / 2
        edge_length = np.linalg.norm(adjusted_points[i] - adjusted_points[j])
        plt.text(mid_x, mid_y, f"{edge_length:.2f}", fontsize=10, color='green')

    plt.title(f"Scaled Polygon (Edge Total = {total_length:.2f})")
    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()