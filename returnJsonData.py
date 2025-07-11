import json
import ast
import pandas as pd
import math

def export_graph_data(input_points, stations, setiners, intermediate_nodes, commodities):
    df = pd.read_csv("data.csv")
    y_ij = ast.literal_eval(df["yij_1a"].iloc[-1])
    f_ij = ast.literal_eval(df["fij_1a"].iloc[-1])

    # Nodes
    nodes = []
    for idx, (x, y) in enumerate(input_points):
        if idx < len(stations):
            node_type = "station"
        elif idx < len(stations) + len(setiners):
            node_type = "steiner"
        elif idx < len(stations) + len(setiners) + len(intermediate_nodes):
            node_type = "intermediate"
        else:
            node_type = "unknown"

        nodes.append({
            "id": idx,
            "x": x,
            "y": y,
            "type": node_type
        })

    # Edges (y_ij = 1)
    edges = []
    for (i, j), val in y_ij.items():
        if val == 1:
            length = math.hypot(input_points[i][0] - input_points[j][0],
                                input_points[i][1] - input_points[j][1])
            edges.append({
                "source": i,
                "target": j,
                "length": round(length, 2)
            })

    # Flows (f_ij)
    flows = []
    for (k, i, j), flow in f_ij.items():
        if flow > 0:
            flows.append({
                "commodity": k,
                "source": i,
                "target": j,
                "flow": flow
            })

    # Final output structure
    result = {
        "nodes": nodes,
        "edges": edges,
        "flows": flows
    }

    # Optional: save to JSON file
    with open("graph_output.json", "w") as f:
        json.dump(result, f, indent=4)

    return result
