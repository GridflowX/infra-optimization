from gurobipy import Model, GRB, quicksum
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import pulp
import random

def minimiseCostWithAnnotations(points, stc, stic, ec):    
    global stations, n_stations, n_steiner, total_points, x, y, d, z, edges
    stations = points
    n_stations = len(stations)
    n_steiner = 1  # Number of Steiner points
    total_points = n_stations + n_steiner

    # Initial edges
    initial_edges = [(i, (i + 1) % n_stations) for i in range(n_stations)]

    # Create a model
    model = Model("Steiner Tree with Costs")

    # Variablesfor
    x = model.addVars(total_points, vtype=GRB.CONTINUOUS, name="x")  # x-coordinates
    y = model.addVars(total_points, vtype=GRB.CONTINUOUS, name="y")  # y-coordinates
    d = model.addVars(total_points, total_points, vtype=GRB.CONTINUOUS, name="d")  # Squared distance
    z = model.addVars(total_points, total_points, vtype=GRB.CONTINUOUS, name="z")  # Distance (linearized)
    edges = model.addVars(total_points, total_points, vtype=GRB.BINARY, name="edges")  # Edge existence

    # Costs
    station_cost = stc  # in lakhs
    steiner_cost = stic  # in lakhs
    edge_cost_per_unit = ec  # in lakhs per km

    # Calculate initial total cost
    total_station_cost = n_stations * station_cost
    initial_edges = [(i, (i + 1) % n_stations) for i in range(n_stations)]
    initial_edge_cost = sum(edge_cost_per_unit * np.sqrt((stations[i][0] - stations[j][0]) ** 2 + (stations[i][1] - stations[j][1]) ** 2) for i, j in initial_edges)
    initial_cost = total_station_cost + initial_edge_cost

    print("Initial Total Cost:", initial_cost)

    # Update changes in model
    model.update()

    # Objective: Minimize total cost
    model.setObjective(
        (quicksum(station_cost * (i <= n_stations) for i in range(total_points)) +  # Station cost
        quicksum(steiner_cost * (i >= n_stations) for i in range(total_points)) +  # Steiner points cost
        quicksum(edge_cost_per_unit * z[i, j] * edges[i, j] for i in range(total_points) for j in range(i))),  # Edge cost
        GRB.MINIMIZE
    )

    # Constraints
    M = 1000  # A sufficiently large constant
    for i in range(total_points):
        for j in range(total_points):
            if i != j:
                # Squared distance calculation
                model.addConstr(d[i, j] >= (x[i] - x[j]) * (x[i] - x[j]) + (y[i] - y[j]) * (y[i] - y[j]))

                # Ensure distance z[i, j] is 0 if edge is not selected
                model.addConstr(z[i, j] <= M * edges[i, j])

                # Approximation: Relate z[i, j] to d[i, j]
                model.addConstr(z[i, j] >= d[i, j])

    # Fix station coordinates
    for i in range(n_stations):
        model.addConstr(x[i] == stations[i][0])
        model.addConstr(y[i] == stations[i][1])

    # Connectivity constraints
    for i in range(n_stations):
        model.addConstr(quicksum(edges[i, j] for j in range(total_points) if i != j) >= 1)

    # Cost constraint optimized value should be less than initial value
    # model.addConstr(
    #     (quicksum(station_cost * (i <= n_stations) for i in range(total_points)) +  # Station cost
    #     quicksum(steiner_cost * (i >= n_stations) for i in range(total_points)) + # Steiner Cost
    #     quicksum(edge_cost_per_unit * z[i, j] * edges[i, j] for i in range(total_points) for j in range(i))) <= (initial_cost-0.00000000001), "cost_constraint"
    # )

   #Create auxiliary variables for distances
    d = {}
    for i in range(n_stations, total_points):  # New points
        for j in range(n_stations):  # Existing stations
            d[i, j] = model.addVar(lb=0, name=f"d_{i}_{j}")

    # Add Second-Order Cone (SOC) Constraints
    for i in range(n_stations, total_points):  # New points
        for j in range(n_stations):  # Existing stations
            model.addConstr(d[i, j] * d[i, j] >= (x[i] - stations[j][0])**2 + (y[i] - stations[j][1])**2)

    # Total distance constraint
    model.addConstr(
        station_cost*n_stations + steiner_cost*n_steiner + edge_cost_per_unit*quicksum(d[i, j] for i in range(n_stations, total_points) for j in range(n_stations)) <= (initial_cost - 0.000001)
    )


    # Solve the model
    model.optimize()

    # Helper function to calculate distances
    def calculate_distance(x1, y1, x2, y2):
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    if model.status == GRB.OPTIMAL:
        print("Optimal solution found.")
        print("Objective value:", model.objVal)
        # Extract solution
        solution_x = [stations[i][0] if i < n_stations else x[i].X for i in range(total_points)]
        solution_y = [stations[i][1] if i < n_stations else y[i].X for i in range(total_points)]
        selected_edges = [(i, j) for i in range(i) for j in range(total_points) if edges[i, j].X >= 0.0 and (i >= n_stations or j >= n_stations)]
        print(f"---------------------selected_edges---------{selected_edges}------------")

        # Calculate costs
        total_station_cost = n_stations * station_cost
        total_steiner_cost = sum(steiner_cost for i in range(n_stations, total_points))
        total_edge_cost = sum(edge_cost_per_unit * np.sqrt((solution_x[i] - solution_x[j]) ** 2 + (solution_y[i] - solution_y[j]) ** 2) for i, j in selected_edges)
        total_cost = total_station_cost + total_steiner_cost + total_edge_cost

        print(f"\n----in steiner prob total_cost: {total_cost} ----------\n")

        # Calculate initial edge cost for the fully connected stations
        initial_edges = [(i, (i + 1) % n_stations) for i in range(n_stations)]
        initial_edge_cost = sum(edge_cost_per_unit * np.sqrt((stations[i][0] - stations[j][0]) ** 2 + (stations[i][1] - stations[j][1]) ** 2) for i, j in initial_edges)
        initial_cost = total_station_cost + initial_edge_cost

        # Plotting the graphs
        fig, axs = plt.subplots(1, 2, figsize=(16, 8))

        # Initial Graph
        axs[0].scatter([s[0] for s in stations], [s[1] for s in stations], c="blue", label="Stations")
        for i, j in initial_edges:
            axs[0].plot([stations[i][0], stations[j][0]], [stations[i][1], stations[j][1]])
            # Add distance annotation for initial edges
            dist = np.sqrt((stations[i][0] - stations[j][0])**2 + (stations[i][1] - stations[j][1])**2)
            axs[0].text((stations[i][0] + stations[j][0]) / 2,
                        (stations[i][1] + stations[j][1]) / 2,
                        f"{dist:.2f}", color="green", fontsize=8)

        axs[0].set_title(f"Initial Configuration\nTotal Cost: {initial_cost:.2f} lakhs")
        axs[0].legend()

        # Display cost breakdown in the initial graph
        axs[0].text(0.05, 0.95,
                    f"Station Cost: {total_station_cost:.2f}L\n"
                    f"Edge Cost: {initial_edge_cost:.2f}L",
                    transform=axs[0].transAxes, fontsize=10, verticalalignment="top", bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

        # Optimized Solution with annotations
        axs[1].scatter([solution_x[i] for i in range(n_stations)],
                      [solution_y[i] for i in range(n_stations)], c="blue", label="Stations")
        axs[1].scatter([solution_x[i] for i in range(n_stations, total_points)],
                      [solution_y[i] for i in range(n_stations, total_points)], c="red", label="Steiner Points")

        for i, j in selected_edges:
            axs[1].plot([solution_x[i], solution_x[j]], [solution_y[i], solution_y[j]], "k-", alpha=0.7)
            # Add distance annotation for optimized edges
            dist = calculate_distance(solution_x[i], solution_y[i], solution_x[j], solution_y[j])
            axs[1].text((solution_x[i] + solution_x[j]) / 2,
                        (solution_y[i] + solution_y[j]) / 2,
                        f"{dist:.2f}", color="green", fontsize=8)

        axs[1].set_title(f"Optimized Solution\nTotal Cost: {total_cost:.2f} lakhs")
        axs[1].legend()

        # Display cost breakdown in the optimized graph
        axs[1].text(0.05, 0.95,
                    f"Station Cost: {total_station_cost:.2f}L\n"
                    f"Steiner Cost: {total_steiner_cost:.2f}L\n"
                    f"Edge Cost: {total_edge_cost:.2f}L",
                    transform=axs[1].transAxes, fontsize=10, verticalalignment="top", bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

        plt.show()

        return total_cost, solution_x, solution_y, selected_edges
    else:
        print("No optimal solution found.")
        # Calculate initial edge cost for the fully connected stations
        solution_x = [stations[i][0] for i in range(n_stations)]
        solution_y = [stations[i][1] for i in range(n_stations)]
        total_station_cost = n_stations * station_cost
        initial_edges = [(i, (i + 1) % n_stations) for i in range(n_stations)]
        initial_edge_cost = sum(edge_cost_per_unit * np.sqrt((stations[i][0] - stations[j][0]) ** 2 + (stations[i][1] - stations[j][1]) ** 2) for i, j in initial_edges)
        initial_cost = total_station_cost + initial_edge_cost

        # Plotting the graphs
        fig, ax = plt.subplots(figsize=(8, 6))

        # Initial Graph
        ax.scatter([s[0] for s in stations], [s[1] for s in stations], c="blue", label="Stations")
        for i, j in initial_edges:
            ax.plot([stations[i][0], stations[j][0]], [stations[i][1], stations[j][1]])
            # Add distance annotation for initial edges
            dist = np.sqrt((stations[i][0] - stations[j][0])**2 + (stations[i][1] - stations[j][1])**2)
            ax.text((stations[i][0] + stations[j][0]) / 2,
                        (stations[i][1] + stations[j][1]) / 2,
                        f"{dist:.2f}", color="green", fontsize=8)

        ax.set_title(f"Initial Configuration which is better than optimized\nTotal Cost: {initial_cost:.2f} lakhs")
        ax.legend()

        # Display cost breakdown in the initial graph
        ax.text(0.05, 0.95,
                    f"Station Cost: {total_station_cost:.2f}L\n"
                    f"Edge Cost: {initial_edge_cost:.2f}L",
                    transform=ax.transAxes, fontsize=10, verticalalignment="top", bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))
        plt.show()

        return initial_cost, solution_x, solution_y, initial_edges

# Scaling of points function
# Function to calculate total polygon perimeter
def polygon_perimeter(points):
    return sum(np.linalg.norm(points[i] - points[(i + 1) % len(points)]) for i in range(len(points)))

# Objective function (dummy, as we care only about constraints)
def objective(scale):
    return 0  # We only care about satisfying constraints

# Constraint: Total perimeter of the polygon must be 1
def perimeter_constraint(scale, points):
    scaled_points = points * scale
    return polygon_perimeter(scaled_points) - 1

# Adjust points to ensure the total perimeter is 1
def adjust_polygon_perimeter(points):
    points = np.array(points)

    # Initial guess for the scaling factor
    initial_scale = 1.0

    # Define the constraint for the total perimeter
    constraints = {'type': 'eq', 'fun': lambda scale: perimeter_constraint(scale, points)}

    # Minimize with the constraint
    result = minimize(objective, [initial_scale], method='SLSQP', constraints=constraints, bounds=[(0, None)])

    # Apply the optimized scaling factor
    optimized_scale = result.x[0]
    optimized_points = points * optimized_scale

    return optimized_points, polygon_perimeter(optimized_points)

def inputs(adjusted_points, total_perimeter, edges_input):
  # Adjust points to achieve total perimeter of 1
  print("Adjusted Points:")
  print(adjusted_points)
  print("Total Perimeter:", total_perimeter)

  # Plot the adjusted polygon
  adjusted_points = np.array(adjusted_points)
  edges = edges_input
  plt.figure(figsize=(8, 8))
  plt.scatter(adjusted_points[:, 0], adjusted_points[:, 1], color='blue', label='Adjusted Points')
  for i, (x, y) in enumerate(adjusted_points):
      plt.text(x, y, f"P{i}", fontsize=12, ha='right', color='red')

  # # Connect points to show the polygon
  # for i in range(len(adjusted_points)):
  #     j = (i + 1) % len(adjusted_points)
  #     plt.plot([adjusted_points[i, 0], adjusted_points[j, 0]],
  #             [adjusted_points[i, 1], adjusted_points[j, 1]], 'k--', alpha=0.6)
  #     # Display edge lengths
  #     mid_x = (adjusted_points[i, 0] + adjusted_points[j, 0]) / 2
  #     mid_y = (adjusted_points[i, 1] + adjusted_points[j, 1]) / 2
  #     edge_length = np.linalg.norm(adjusted_points[i] - adjusted_points[j])
  #     plt.text(mid_x, mid_y, f"{edge_length:.2f}", fontsize=10, color='green')

  # Plot edges based on given edge list
  for edge in edges:
    i, j = edge
    plt.plot([adjusted_points[i, 0], adjusted_points[j, 0]],
             [adjusted_points[i, 1], adjusted_points[j, 1]], 'k--', alpha=0.6)
    
    # Display edge lengths
    mid_x = (adjusted_points[i, 0] + adjusted_points[j, 0]) / 2
    mid_y = (adjusted_points[i, 1] + adjusted_points[j, 1]) / 2
    edge_length = np.linalg.norm(adjusted_points[i] - adjusted_points[j])
    plt.text(mid_x, mid_y, f"{edge_length:.2f}", fontsize=10, color='green')

  plt.title("Adjusted Polygon with Total Perimeter of 1")
  plt.xlabel("X-coordinate")
  plt.ylabel("Y-coordinate")
  plt.axis('equal')
  plt.grid(True)
  plt.legend()
  plt.show()