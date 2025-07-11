from scale_graph import adjust_polygon_to_edge_length, plot_scaled_polygon
from solution_zub_and_zlb import get_zub_zlb
from subgradient_lambda import update_lambda
from data_store_csv import write_to_csv
import numpy as np
import random
from plots import yij_graph, fij_graph, individual_fij_graph, plot_bounds_vs_iteration, plot_gap_vs_iteration

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

    yij_graph(input_points, stations, setiners, intermediate_nodes)
    fij_graph(input_points, commodities, stations, setiners, intermediate_nodes)
    plot_bounds_vs_iteration()
    plot_gap_vs_iteration()
    individual_fij_graph(input_points, commodities,stations, setiners, intermediate_nodes)


if __name__ == "__main__":
    # input points for the graph as coordinates (x, y)
    stations = [(0, 0), (1, 2), (2, 0)]
    setiners = [(1, 0.66)]
    intermediate_nodes = []

    # Total arcs (i, j) where i, j are nodes i.e 0th, 1st , 2nd etc... and i->j is the direction of the arc and only one direction is considered
    edges_input = [(0, 1), (0, 2), (1, 2), (0, 3), (3, 1), (3, 2)]

    # Initialize the costs
    edge_cost = 1

    speed = 30
    alpha = 0.5
    beta = 0.5
    stop_criteria = 1e-3
    scale_factor = 1

    # Capacity of the edges
    capacity = {
        (0, 1) : 1000,
        (0, 2) : 1000,
        (1, 2) : 1000,
        (0, 3) : 1000,
        (1, 3) : 1000,
        (2, 3) : 1000
    }

    # Commodities are the pairs of nodes (i, j) with a demand i.e (source, destination, demand)
    commodities = {
        0 : (0, 1, 10), 
        1 : (0, 2, 10), 
        2 : (1, 0, 10),
        3 : (1, 2, 10),
        4 : (2, 0, 10),
        5 : (2, 1, 10),
    }

    main(stations, setiners, intermediate_nodes, edges_input, edge_cost, speed, alpha, beta, stop_criteria, capacity, commodities, scale_factor)