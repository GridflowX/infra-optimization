import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import ast  # To safely parse dictionary strings

def yij_graph(input_points):

    # Load CSV file
    df = pd.read_csv("data.csv")

    # Convert the y_ij column (string representation of dict) into an actual dictionary
    y_ij = df["yij_1a"].iloc[-1]  # Assuming dictionary is stored in a single row
    y_ij = ast.literal_eval(y_ij)  # Convert string to dictionary safely

    print("y_ij:", y_ij)

    # Given node coordinates (modify based on your data)
    nodes_coordinates = {idx: (x, y) for idx, (x, y) in enumerate(input_points)} 

    # Create graph
    G = nx.DiGraph()
    G.add_nodes_from(nodes_coordinates.keys())

    # Add edges where y_ij[(i, j)] == 1
    for (i, j), val in y_ij.items():
        if val == 1:
            G.add_edge(i, j)

    # Draw the graph
    plt.figure(figsize=(8, 6))
    nx.draw(G, pos=nodes_coordinates, with_labels=True, node_size=500, node_color="lightblue", edge_color="gray", arrows=True, connectionstyle="arc3,rad=0.1")
    plt.title("y_ij Graph")
    plt.show()

def fij_graph(input_points, commodities):

    # Load CSV file
    df = pd.read_csv("data.csv")

    # Convert columns containing dictionaries from strings to actual dictionaries
    y_ij = ast.literal_eval(df["yij_1a"].iloc[-1])  # (i, j) : binary_val
    f_ij = ast.literal_eval(df["fij_1a"].iloc[-1])  # (k, i, j) : flow_value

    # Given node coordinates
    nodes_coordinates = {idx: (x, y) for idx, (x, y) in enumerate(input_points)} 

    # Create a directed graph
    G = nx.DiGraph()

    # Add nodes with positions
    G.add_nodes_from(nodes_coordinates.keys())

    # Define color map for commodities (optional)
    commodity_colors = {k: color for k, color in zip(commodities.keys(), ['r', 'g', 'b', 'm', 'c'])}

    # Add edges based on flow values
    edge_labels = {}
    for (k, i, j), flow in f_ij.items():
        if flow > 0:  # Only consider active flows
            G.add_edge(i, j, weight=flow, color=commodity_colors.get(k, "gray"))
            edge_labels[(i, j)] = f"{flow}"  # Label edges with flow value

    # Extract edge colors and widths
    edges = G.edges(data=True)
    edge_colors = [data["color"] for _, _, data in edges]
    edge_widths = [data["weight"] * 0.1 for _, _, data in edges]  # Scale widths for visibility

    # Plot graph
    plt.figure(figsize=(8, 6))
    nx.draw(
        G, pos=nodes_coordinates, with_labels=True, node_size=500, 
        node_color="lightblue", edge_color=edge_colors, width=edge_widths, arrows=True
    )
    nx.draw_networkx_edge_labels(G, pos=nodes_coordinates, edge_labels=edge_labels, font_size=10)

    # Show plot
    plt.title("Flow Network Visualization")
    plt.show()

def individual_fij_graph(input_points, commodities):
    # Load CSV file
    df = pd.read_csv("data.csv")

    # Convert columns containing dictionaries from strings to actual dictionaries
    y_ij = ast.literal_eval(df["yij_1a"].iloc[-1])  # (i, j) : binary_val
    f_ij = ast.literal_eval(df["fij_1a"].iloc[-1])  # (k, i, j) : flow_value

    # Given node coordinates
    nodes_coordinates = {idx: (x, y) for idx, (x, y) in enumerate(input_points)}

    # Define color map for commodities
    commodity_colors = {k: color for k, color in zip(commodities.keys(), ['r', 'g', 'b', 'm', 'c'])}

    # Get list of commodities
    unique_commodities = set(k for (k, _, _), _ in f_ij.items())

    # Iterate over each commodity and create a separate graph
    for k in unique_commodities:
        G = nx.DiGraph()

        # Add nodes
        G.add_nodes_from(nodes_coordinates.keys())

        # Add edges for the current commodity
        edge_labels = {}
        edge_colors = []
        edge_widths = []

        for (k_flow, i, j), flow in f_ij.items():
            if k_flow == k and flow > 0:  # Only consider edges for the current commodity
                G.add_edge(i, j, weight=flow, color=commodity_colors.get(k, "gray"))
                edge_labels[(i, j)] = f"{flow}"
                edge_colors.append(commodity_colors.get(k, "gray"))
                edge_widths.append(flow * 0.1)  # Scale widths for visibility

        # Create a new figure for each commodity
        plt.figure(figsize=(8, 6))
        plt.title(f"Flow Network for Commodity {k}")
        
        nx.draw(
            G, pos=nodes_coordinates, with_labels=True, node_size=500,
            node_color="lightblue", edge_color=edge_colors, width=edge_widths,
            arrows=True
        )
        nx.draw_networkx_edge_labels(G, pos=nodes_coordinates, edge_labels=edge_labels, font_size=10)

        # Show each plot separately
        plt.show()