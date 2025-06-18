import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import ast
import matplotlib.patches as mpatches
import math

def yij_graph(input_points, stations, setiners, intermediate_nodes):
    # Load CSV file
    df = pd.read_csv("data.csv")
    y_ij = ast.literal_eval(df["yij_1a"].iloc[-1])

    # Node positions
    nodes_coordinates = {idx: (x, y) for idx, (x, y) in enumerate(input_points)}
    G = nx.DiGraph()
    G.add_nodes_from(nodes_coordinates.keys())

    total_length = 0
    edge_labels = {}
    seen_pairs = set()

    # Add edges and compute total length
    for (i, j), val in y_ij.items():
        if val == 1:
            G.add_edge(i, j)

            # Only add length once per undirected pair
            undirected_key = tuple(sorted((i, j)))
            if undirected_key not in seen_pairs:
                seen_pairs.add(undirected_key)
                x1, y1 = nodes_coordinates[i]
                x2, y2 = nodes_coordinates[j]
                length = math.hypot(x2 - x1, y2 - y1)
                total_length += length
                edge_labels[(i, j)] = f"{length:.2f}"
            else:
                # Don't display label again for reverse edge
                edge_labels[(i, j)] = ""

    # Node colors
    node_colors = []
    station_range = range(len(stations))
    steiner_range = range(len(stations), len(stations) + len(setiners))
    intermediate_range = range(len(stations) + len(setiners), len(stations) + len(setiners) + len(intermediate_nodes))

    for i in range(len(input_points)):
        if i in station_range:
            node_colors.append("blue")
        elif i in steiner_range:
            node_colors.append("red")
        elif i in intermediate_range:
            node_colors.append("teal")
        else:
            node_colors.append("gray")

    # Draw graph
    plt.figure(figsize=(10, 8))
    nx.draw(G, pos=nodes_coordinates, with_labels=True, node_size=500,
            node_color=node_colors, edge_color="gray", arrows=True,
            connectionstyle="arc3,rad=0.1")

    # Draw edge labels (only one per edge pair)
    nx.draw_networkx_edge_labels(G, pos=nodes_coordinates, edge_labels=edge_labels, font_size=9)

    # Legend
    legend_handles = []
    if station_range: legend_handles.append(mpatches.Patch(color='blue', label='Stations'))
    if steiner_range: legend_handles.append(mpatches.Patch(color='red', label='Steiner Points'))
    if intermediate_range: legend_handles.append(mpatches.Patch(color='black', label='Intermediate Nodes'))
    plt.legend(handles=legend_handles)

    # Title with total length
    plt.title(f"y_ij Graph — Total Length: {total_length:.2f}")
    plt.show()
    #return G

def fij_graph(input_points, commodities, stations, setiners, intermediate_nodes):
    df = pd.read_csv("data.csv")
    y_ij = ast.literal_eval(df["yij_1a"].iloc[-1])
    f_ij = ast.literal_eval(df["fij_1a"].iloc[-1])

    nodes_coordinates = {idx: (x, y) for idx, (x, y) in enumerate(input_points)}
    G = nx.DiGraph()
    G.add_nodes_from(nodes_coordinates.keys())

    commodity_colors = {k: color for k, color in zip(commodities.keys(), ['r', 'g', 'b', 'm', 'c'])}

    edge_labels = {}
    for (k, i, j), flow in f_ij.items():
        if flow > 0:
            G.add_edge(i, j, weight=flow, color=commodity_colors.get(k, "gray"))
            edge_labels[(i, j)] = f"{flow}"

    edges = G.edges(data=True)
    edge_colors = [data["color"] for _, _, data in edges]
    edge_widths = [data["weight"] * 0.1 for _, _, data in edges]

    # Node color logic
    node_colors = []
    station_range = range(len(stations))
    steiner_range = range(len(stations), len(stations) + len(setiners))
    intermediate_range = range(len(stations) + len(setiners), len(stations) + len(setiners) + len(intermediate_nodes))

    for i in range(len(input_points)):
        if i in station_range:
            node_colors.append("blue")
        elif i in steiner_range:
            node_colors.append("red")
        elif i in intermediate_range:
            node_colors.append("teal")
        else:
            node_colors.append("gray")

    # Draw
    plt.figure(figsize=(8, 6))
    nx.draw(G, pos=nodes_coordinates, with_labels=True, node_size=500,
            node_color=node_colors, edge_color=edge_colors,
            width=edge_widths, arrows=True)
    nx.draw_networkx_edge_labels(G, pos=nodes_coordinates, edge_labels=edge_labels, font_size=10)

    legend_handles = []
    if station_range: legend_handles.append(mpatches.Patch(color='blue', label='Stations'))
    if steiner_range: legend_handles.append(mpatches.Patch(color='red', label='Steiner Points'))
    if intermediate_range: legend_handles.append(mpatches.Patch(color='black', label='Intermediate Nodes'))
    plt.legend(handles=legend_handles)

    plt.title("Flow Network Visualization")
    plt.show()

    #return G, edge_labels


def individual_fij_graph(input_points, commodities, stations, setiners, intermediate_nodes):
    df = pd.read_csv("data.csv")
    y_ij = ast.literal_eval(df["yij_1a"].iloc[-1])
    f_ij = ast.literal_eval(df["fij_1a"].iloc[-1])

    nodes_coordinates = {idx: (x, y) for idx, (x, y) in enumerate(input_points)}
    commodity_colors = {k: color for k, color in zip(commodities.keys(), ['r', 'g', 'b', 'm', 'c'])}
    unique_commodities = set(k for (k, _, _), _ in f_ij.items())

    graphs = {}

    for k in unique_commodities:
        G = nx.DiGraph()
        G.add_nodes_from(nodes_coordinates.keys())

        edge_labels = {}
        edge_colors = []
        edge_widths = []

        for (k_flow, i, j), flow in f_ij.items():
            if k_flow == k and flow > 0:
                G.add_edge(i, j, weight=flow, color=commodity_colors.get(k, "gray"))
                edge_labels[(i, j)] = f"{flow}"
                edge_colors.append(commodity_colors.get(k, "gray"))
                edge_widths.append(flow * 0.1)

        # Node color logic
        node_colors = []
        station_range = range(len(stations))
        steiner_range = range(len(stations), len(stations) + len(setiners))
        intermediate_range = range(len(stations) + len(setiners), len(stations) + len(setiners) + len(intermediate_nodes))

        for i in range(len(input_points)):
            if i in station_range:
                node_colors.append("blue")
            elif i in steiner_range:
                node_colors.append("red")
            elif i in intermediate_range:
                node_colors.append("teal")
            else:
                node_colors.append("gray")

        # Draw
        plt.figure(figsize=(8, 6))
        plt.title(f"Flow Network for Commodity {k}")
        nx.draw(
            G, pos=nodes_coordinates, with_labels=True, node_size=500,
            node_color=node_colors, edge_color=edge_colors,
            width=edge_widths, arrows=True
        )
        nx.draw_networkx_edge_labels(G, pos=nodes_coordinates, edge_labels=edge_labels, font_size=10)

        legend_handles = []
        if station_range: legend_handles.append(mpatches.Patch(color='blue', label='Stations'))
        if steiner_range: legend_handles.append(mpatches.Patch(color='red', label='Steiner Points'))
        if intermediate_range: legend_handles.append(mpatches.Patch(color='black', label='Intermediate Nodes'))
        plt.legend(handles=legend_handles)

        plt.show()
        graphs[k] = (G, edge_labels)

    #return graphs

def plot_bounds_vs_iteration():
    df = pd.read_csv("data.csv")

    x = df["Iteration"]
    y_lb = df["ZLB"]
    y_ub = df["ZUB"]

    plt.figure(figsize=(10, 6))
    plt.plot(x, y_lb , label="ZLB (Lower Bound)", marker='o', color='orange')
    plt.plot(x, y_ub, label="ZUB (Upper Bound)", marker='s', color='blue')

    plt.xlabel("Iteration")
    plt.ylabel("Objective Value")
    plt.title("ZLB and ZUB vs Iteration")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

def plot_gap_vs_iteration():
    df = pd.read_csv("data.csv")

    x = df["Iteration"]
    y_gap = df["GAP"]

    plt.figure(figsize=(10, 6))
    plt.plot(x, y_gap, label="Gap", marker='^', color='green')

    plt.xlabel("Iteration")
    plt.ylabel("Gap")
    plt.title("Gap vs Iteration")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

