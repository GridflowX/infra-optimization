from scale_graph import adjust_polygon_to_edge_length, plot_scaled_polygon
from solution_zub_and_zlb import get_zub_zlb
from subgradient_lambda import update_lambda
from data_store_csv import write_to_csv
import numpy as np
import random
from plots import yij_graph, fij_graph, individual_fij_graph, plot_bounds_vs_iteration, plot_gap_vs_iteration
import json
import os
from returnJsonData import export_graph_data

def main(stations, setiners, intermediate_nodes, edges_input, edge_cost, speed, alpha, beta, stop_criteria, capacity, commodities, scale_factor):
    # Total nodes including stations, setiners and intermediate nodes
    input_points = stations + setiners + intermediate_nodes

    #scale the input graph to have overall distance to target lenth
    adjusted_points, total_perimeter = adjust_polygon_to_edge_length(input_points, edges_input, scale_factor)

    #plot the scaled graph
    plot_scaled_polygon(adjusted_points, total_perimeter, edges_input, stations, setiners, intermediate_nodes)

    # Making both direction arcs
    edges_input += [(j, i) for (i, j) in edges_input]

    # Convert the points to a list of tuples
    optimized_points = [tuple(point) for point in adjusted_points]

    # Making the capacity dictionary symmetric
    for (i, j), cap in list(capacity.items()):
        if (j, i) not in capacity:
            capacity[(j, i)] = cap


    # Initialize the lambda values for each commodity and edge
    lambda_k = {(k, i, j) : random.random() for k in commodities for (i, j) in edges_input}
    lambda_n = np.array(list(lambda_k.values()))

    data = []
    theta_n = 0
    s_lambda = 0
    n = 0

    while True:
        zub, zlb, fij, yij, y_ub, f_ub = get_zub_zlb(optimized_points, stations, edge_cost, lambda_k, speed, capacity, alpha, beta, edges_input, commodities)
        
        gap = zub - zlb

        print()
        print(f"---------------- Iteration {n} ----------------")
        print(f"ZLB: {zlb}")
        print(f"ZUB: {zub}")
        print(f"GAP: {gap}")
        print(f"------------------------------------------------")

        iteration_wise_data = {"Iteration":n, "ZLB":zlb, "ZUB":zub, "GAP":gap, "Lambda":lambda_n, "theta_n":theta_n, "fij":fij, "yij":yij, "s_lambda":s_lambda, "yij_1a": y_ub, "fij_1a": f_ub}
        data.append(iteration_wise_data)
        write_to_csv(data)
        print()
        
        if gap <= stop_criteria:
            print("------------- Stopping Criteria Met ------------")
            print(f"Iterations: {n}")
            print(f"Final ZLB: {zlb}")
            print(f"Final ZUB: {zub}")
            print(f"Final GAP: {gap}")
            print(f"------------------------------------------------")
            final_data = {"Iteration":f"Final Iteration:{n+1}", "ZLB":f"{zlb}", "ZUB":f"{zub}", "GAP":f"{gap}", "Lambda":lambda_n, "theta_n":theta_n, "fij":fij, "yij":yij, "s_lambda":s_lambda, "yij_1a": y_ub, "fij_1a": f_ub}
            data.append(final_data)
            break
        else:
            n += 1

            lambda_k, s_lambda_dict, theta_n = update_lambda(lambda_k, fij, yij, zub, zlb, commodities, edges_input, n)
            lambda_n = np.array(list(lambda_k.values()))
            s_lambda = np.array(list(s_lambda_dict.values()))

    write_to_csv(data)

    # Export graph data to JSON
    json_file = export_graph_data(input_points, stations, setiners, intermediate_nodes, commodities)
    print(json.dumps(json_file, indent=2))

    # Save to file (both locations for compatibility)
    with open("json_output.json", "w") as f:
        json.dump(json_file, f, indent=4)
    
    # Also save to backend directory if it exists
    if os.path.exists("./backend"):
        with open("./backend/json_output.json", "w") as f:
            json.dump(json_file, f, indent=4)

    # Plotting graphs
    yij_graph(input_points, stations, setiners, intermediate_nodes)
    fij_graph(input_points, commodities, stations, setiners, intermediate_nodes)
    plot_bounds_vs_iteration()
    plot_gap_vs_iteration()
    individual_fij_graph(input_points, commodities,stations, setiners, intermediate_nodes)


if __name__ == "__main__":
    # Load and parse the JSON
    with open("input_hack.json", "r") as f:
        data = json.load(f)

    # Convert lists to tuples where needed
    stations = [tuple(pt) for pt in data["stations"]]
    steiners = [tuple(pt) for pt in data["steiners"]]
    intermediate_nodes = [tuple(pt) for pt in data["intermediate_nodes"]]
    edges_input = [tuple(edge) for edge in data["edges_input"]]
    capacity = {tuple(json.loads(k)): v for k, v in data["capacity"].items()}
    commodities = {int(k): tuple(v) for k, v in data["commodities"].items()}

    # Extract remaining parameters
    edge_cost = data["edge_cost"]
    speed = data["speed"]
    # alpha = float(os.environ.get("ALPHA", 0))
    # beta = float(os.environ.get("BETA", 0))
    alpha = data["alpha"]
    beta = data["beta"]
    stop_criteria = data["stop_criteria"]
    scale_factor = data["scale_factor"]

    main(stations, steiners, intermediate_nodes, edges_input, edge_cost, speed, alpha, beta, stop_criteria, capacity, commodities, scale_factor)