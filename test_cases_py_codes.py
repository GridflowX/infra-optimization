#------------------1st-------------------
from steiner_solution import adjust_polygon_perimeter, inputs, minimiseCostWithAnnotations
from solution_zub_and_zlb import get_zub_zlb
from subgradient_lambda import update_lambda
from data_store_csv import write_to_csv
import numpy as np
import random
from plots import yij_graph, fij_graph, individual_fij_graph

def get_new_lambda(lambda_n, fij, yij, zub, zlb, commodities, arcs, n):
    lambda_new, theta_n, s_lambda = update_lambda(lambda_n, fij, yij, zub, zlb, commodities, arcs, n)
    return lambda_new, theta_n, s_lambda

# Example input points (initial polygon)
input_points = [(0,1),(0,2), (1,3),(2,2),(2,1),(1,0),(1, 1.5)]
adjusted_points, total_perimeter = adjust_polygon_perimeter(input_points)
edges_input = [(0, 1), (1, 0), (1, 2), (2, 1), (2, 3), (3, 2), (3, 4), (4, 3), (4, 5), (5, 4), (5, 0), (0, 5), (0, 6), (6, 0), (1, 6), (6, 1), (2, 6), (6, 2), (3, 6), (6, 3),(4, 6), (6, 4), (5, 6),(6, 5)]
inputs(adjusted_points,total_perimeter, edges_input)

# Convert the points to a list of tuples
optimized_points = [tuple(point) for point in adjusted_points]
print(optimized_points) 

station_cost = 1
steiner_cost = 6
edge_cost = 1

# cost, solution_x, solution_y, selected_edges = minimiseCostWithAnnotations(optimized_points, station_cost, steiner_cost, edge_cost)
# points_for_zlb_zub = [(solution_x[i], solution_y[i]) for i in range(len(solution_x))]

cost = station_cost #this is dummy currently for testing actaully its the cost from the steiner tree solution

points_for_zlb_zub = optimized_points
selected_edges = edges_input

speed = 30
capacity = 1000
alpha = 0.5
beta = 0.5
stop_criteria = 1e-3
n = 0

sources = [i for i in range(len(optimized_points))]
destinations = [i for i in range(len(optimized_points))]

# # (src, dest, demand) for each commodity
# commodities = {
#     idx + 1: (i, j, 10)  
#     for idx, (i, j) in enumerate((i, j) for i in sources for j in destinations if i != j)
# }

commodities = {
    0 : (0, 1, 10),
    1 : (0, 2, 10),
    2 : (0, 3, 10),
    3 : (0, 4, 10),
    4 : (0, 5, 10)
}

lambda_k = {(k, i, j) : random.random() for k in commodities for (i, j) in selected_edges}
lambda_n = np.array(list(lambda_k.values()))

data = []
theta_n = 0
s_lambda = 0

while True:
    zub, zlb, fij, yij, y_ub, f_ub = get_zub_zlb(points_for_zlb_zub, station_cost, steiner_cost, edge_cost, lambda_k, speed, capacity, alpha, beta, cost, selected_edges, commodities)
    
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
        # print(f"------------------------------------------------")
        # print(f"Iteration {n}: Lambda before update: {lambda_n}")

        lambda_k, s_lambda_dict, theta_n = get_new_lambda(lambda_k, fij, yij, zub, zlb, commodities, selected_edges, n)
        lambda_n = np.array(list(lambda_k.values()))
        s_lambda = np.array(list(s_lambda_dict.values()))

        #lambda_k = {(k, i, j): lambda_n[idx] for idx, (k, i, j) in enumerate(lambda_k.keys())}
        
        # print(f"Iteration {n}: Lambda after update: {lambda_n}")
        # print(f"------------------------------------------------")

write_to_csv(data)

yij_graph(input_points)
fij_graph(input_points, commodities)
individual_fij_graph(input_points, commodities)

#-------------------1st end-------------------


#-------------------2nd-------------------
from steiner_solution import adjust_polygon_perimeter, inputs, minimiseCostWithAnnotations
from solution_zub_and_zlb import get_zub_zlb
from subgradient_lambda import update_lambda
from data_store_csv import write_to_csv
import numpy as np
import random
from plots import yij_graph, fij_graph, individual_fij_graph

def get_new_lambda(lambda_n, fij, yij, zub, zlb, commodities, arcs, n):
    lambda_new, theta_n, s_lambda = update_lambda(lambda_n, fij, yij, zub, zlb, commodities, arcs, n)
    return lambda_new, theta_n, s_lambda

# Example input points (initial polygon)
input_points = [(-0.19921644098254898, 1.0), (0.7490302538367553, -0.692930709088227), (-0.42937163075446505, -0.9163880981997311), (-0.9973329810439262, 0.12065012966431125), (0.8768907989441854, 0.4886686776236478)]
adjusted_points, total_perimeter = adjust_polygon_perimeter(input_points)
edges_input = [(0, 1), (0, 2), (0, 3), (0, 4), (1, 2), (1, 3), (1, 4), (2, 3), (2, 4), (3, 4)]
edges_input += [(b, a) for (a, b) in edges_input]  # Add reverse edges to the list
print(len(edges_input))
inputs(adjusted_points,total_perimeter, edges_input)

# Convert the points to a list of tuples
optimized_points = [tuple(point) for point in adjusted_points]
print(optimized_points) 

station_cost = 1
steiner_cost = 6
edge_cost = 1

# cost, solution_x, solution_y, selected_edges = minimiseCostWithAnnotations(optimized_points, station_cost, steiner_cost, edge_cost)
# points_for_zlb_zub = [(solution_x[i], solution_y[i]) for i in range(len(solution_x))]

cost = station_cost #this is dummy currently for testing actaully its the cost from the steiner tree solution

points_for_zlb_zub = optimized_points
selected_edges = edges_input

speed = 30
capacity = 1000
alpha = 0.5
beta = 0.5
stop_criteria = 1e-3
n = 0

commodities = {
    0 : (0, 1, 10),  #(source, destination, demand)
    1 : (0, 2, 10),
    2 : (0, 3, 10),
    3 : (0, 4, 10),
    4 : (1, 0, 10),
    5 : (1, 2, 10),
}

# commodities = {
#     idx : (i, j, 10) for idx, (i, j) in enumerate((i, j) for i in range(len(optimized_points)-4) for j in range(len(optimized_points)-4) if i != j)
# }

print(f"-----------Total no of commodities: {len(list(commodities.keys()))}-------------")

lambda_k = {(k, i, j) : random.random() for k in commodities for (i, j) in selected_edges}
lambda_n = np.array(list(lambda_k.values()))

data = []
theta_n = 0
s_lambda = 0

while True:
    zub, zlb, fij, yij, y_ub, f_ub = get_zub_zlb(points_for_zlb_zub, station_cost, steiner_cost, edge_cost, lambda_k, speed, capacity, alpha, beta, cost, selected_edges, commodities)
    
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
        # print(f"------------------------------------------------")
        # print(f"Iteration {n}: Lambda before update: {lambda_n}")

        lambda_k, s_lambda_dict, theta_n = get_new_lambda(lambda_k, fij, yij, zub, zlb, commodities, selected_edges, n)
        lambda_n = np.array(list(lambda_k.values()))
        s_lambda = np.array(list(s_lambda_dict.values()))

        #lambda_k = {(k, i, j): lambda_n[idx] for idx, (k, i, j) in enumerate(lambda_k.keys())}
        
        # print(f"Iteration {n}: Lambda after update: {lambda_n}")
        # print(f"------------------------------------------------")

write_to_csv(data)

yij_graph(input_points)
fij_graph(input_points, commodities)
individual_fij_graph(input_points, commodities)

#-------------------2nd end-------------------

#-------------------3rd-------------------
from steiner_solution import adjust_polygon_perimeter, inputs, minimiseCostWithAnnotations
from solution_zub_and_zlb import get_zub_zlb
from subgradient_lambda import update_lambda
from data_store_csv import write_to_csv
import numpy as np
import random
from plots import yij_graph, fij_graph, individual_fij_graph

def get_new_lambda(lambda_n, fij, yij, zub, zlb, commodities, arcs, n):
    lambda_new, theta_n, s_lambda = update_lambda(lambda_n, fij, yij, zub, zlb, commodities, arcs, n)
    return lambda_new, theta_n, s_lambda

# Example input points (initial polygon)
input_points = [(0.005602692257348724, 0.01953447433669544), (-0.916497140057939, -0.018451476422139797), (-0.1861428551180811, 0.1766714103953297), (0.31277166826188735, -0.7386185259460942), (0.01658341061654694, -0.4054790515633216), (-0.669504910062958, -0.4978653577746742), (-0.22157509347890683, -0.6697160363802268), (-0.806673461804519, 0.2917144114161606), (0.550952549045536, -0.036702948111270305), (0.4222514808494576, 0.5389469582381975), (0.4807944396056769, -0.33035262812225386), (-0.6036890426283965, 0.5700911697312979), (-0.24385583781726533, 0.8282374484312651), (0.8589821003316104, 0.37378731472490173), (1.0, -0.10179716295386654), (-0.250919762305275, 0.9014286128198323), (0.4639878836228102, 0.1973169683940732), (-0.687962719115127, -0.6880109593275947), (-0.8838327756636011, 0.7323522915498704)]
adjusted_points, total_perimeter = adjust_polygon_perimeter(input_points)
edges_input = [(0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6), (0, 7), (0, 8), (0, 9), (0, 10), (0, 11), (0, 12), (0, 13), (0, 14), (1, 5), (1, 7), (1, 11), (2, 4), (2, 6), (2, 7), (2, 9), (2, 10), (2, 11), (2, 12), (3, 4), (3, 6), (3, 8), (4, 5), (4, 8), (4, 10), (5, 17), (8, 9), (8, 13), (8, 16), (9, 13), (10, 14), (11, 12), (11, 18), (12, 15), (13, 14)]
edges_input += [(b, a) for (a, b) in edges_input]  # Add reverse edges to the list
inputs(adjusted_points,total_perimeter, edges_input)

# Convert the points to a list of tuples
optimized_points = [tuple(point) for point in adjusted_points]
print(optimized_points) 

station_cost = 1
steiner_cost = 6
edge_cost = 1

# cost, solution_x, solution_y, selected_edges = minimiseCostWithAnnotations(optimized_points, station_cost, steiner_cost, edge_cost)
# points_for_zlb_zub = [(solution_x[i], solution_y[i]) for i in range(len(solution_x))]

cost = station_cost #this is dummy currently for testing actaully its the cost from the steiner tree solution

points_for_zlb_zub = optimized_points
selected_edges = edges_input

speed = 30
capacity = 1000
alpha = 0.5
beta = 0.5
stop_criteria = 1e-3
n = 0

sources = [i for i in range(len(optimized_points))]
destinations = [i for i in range(len(optimized_points))]

# (src, dest, demand) for each commodity
# commodities = {
#     idx + 1: (i, j, 10)  
#     for idx, (i, j) in enumerate((i, j) for i in sources for j in destinations if i != j)
# }

commodities = {
    0 : (0, 1, 10),  #(source, destination, demand)
    1 : (0, 17, 10)
}

lambda_k = {(k, i, j) : random.random() for k in commodities for (i, j) in selected_edges}
lambda_n = np.array(list(lambda_k.values()))

data = []
theta_n = 0
s_lambda = 0

while True:
    zub, zlb, fij, yij, y_ub, f_ub = get_zub_zlb(points_for_zlb_zub, station_cost, steiner_cost, edge_cost, lambda_k, speed, capacity, alpha, beta, cost, selected_edges, commodities)
    
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
        # print(f"------------------------------------------------")
        # print(f"Iteration {n}: Lambda before update: {lambda_n}")

        lambda_k, s_lambda_dict, theta_n = get_new_lambda(lambda_k, fij, yij, zub, zlb, commodities, selected_edges, n)
        lambda_n = np.array(list(lambda_k.values()))
        s_lambda = np.array(list(s_lambda_dict.values()))

        #lambda_k = {(k, i, j): lambda_n[idx] for idx, (k, i, j) in enumerate(lambda_k.keys())}
        
        # print(f"Iteration {n}: Lambda after update: {lambda_n}")
        # print(f"------------------------------------------------")

write_to_csv(data)

yij_graph(input_points)
fij_graph(input_points, commodities)
individual_fij_graph(input_points, commodities)

#-------------------3rd end-------------------

#-------------------4th-------------------
from steiner_solution import adjust_polygon_perimeter, inputs, minimiseCostWithAnnotations
from solution_zub_and_zlb import get_zub_zlb
from subgradient_lambda import update_lambda
from data_store_csv import write_to_csv
import numpy as np
import random
from plots import yij_graph, fij_graph, individual_fij_graph

def get_new_lambda(lambda_n, fij, yij, zub, zlb, commodities, arcs, n):
    lambda_new, theta_n, s_lambda = update_lambda(lambda_n, fij, yij, zub, zlb, commodities, arcs, n)
    return lambda_new, theta_n, s_lambda

# Example input points (initial polygon)
input_points = [(3, 0), (2, 1), (3, 2), (4, 1), (3, 1), (0, 3), (0, 4), (1, 4), (1, 3), (0.5, 3.5), (5, 3), (5, 4), (6, 4), (6, 3), (5.5, 3.5), (3, 2.66)]
adjusted_points, total_perimeter = adjust_polygon_perimeter(input_points)
edges_input = [(0, 1), (1, 2), (2, 3), (3, 0), (1, 4), (2, 4), (3, 4), (0, 4), (2, 8), (8, 10), (10, 2), (2, 15), (8, 15), (10, 15), (5, 6), (6, 7), (7, 8), (8, 5), (5, 9), (6, 9), (7, 9), (8, 9), (7, 11), (10, 11), (11, 12), (12, 13), (13, 10), (10, 14), (11, 14), (12, 14), (13, 14), (5, 1), (13, 3)]
edges_input += [(b, a) for (a, b) in edges_input]  # Add reverse edges
inputs(adjusted_points,total_perimeter, edges_input)

# Convert the points to a list of tuples
optimized_points = [tuple(point) for point in adjusted_points]
print(optimized_points) 

station_cost = 1
steiner_cost = 6
edge_cost = 1

# cost, solution_x, solution_y, selected_edges = minimiseCostWithAnnotations(optimized_points, station_cost, steiner_cost, edge_cost)
# points_for_zlb_zub = [(solution_x[i], solution_y[i]) for i in range(len(solution_x))]

cost = station_cost #this is dummy currently for testing actaully its the cost from the steiner tree solution

points_for_zlb_zub = optimized_points
selected_edges = edges_input

speed = 30
capacity = 1000
alpha = 0.5
beta = 0.5
stop_criteria = 1e-3
n = 0

commodities = {
    0 : (0, 14, 10),  #(source, destination, demand)
    1 : (0, 9, 10),  #works good on demand 4, 5, 7, 9     8.2
}

lambda_k = {(k, i, j) : random.random() for k in commodities for (i, j) in selected_edges}
lambda_n = np.array(list(lambda_k.values()))

data = []
theta_n = 0
s_lambda = 0

while True:
    zub, zlb, fij, yij, y_ub, f_ub = get_zub_zlb(points_for_zlb_zub, station_cost, steiner_cost, edge_cost, lambda_k, speed, capacity, alpha, beta, cost, selected_edges, commodities)
    
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
        # print(f"------------------------------------------------")
        # print(f"Iteration {n}: Lambda before update: {lambda_n}")

        lambda_k, s_lambda_dict, theta_n = get_new_lambda(lambda_k, fij, yij, zub, zlb, commodities, selected_edges, n)
        lambda_n = np.array(list(lambda_k.values()))
        s_lambda = np.array(list(s_lambda_dict.values()))

        #lambda_k = {(k, i, j): lambda_n[idx] for idx, (k, i, j) in enumerate(lambda_k.keys())}
        
        # print(f"Iteration {n}: Lambda after update: {lambda_n}")
        # print(f"------------------------------------------------")

write_to_csv(data)

yij_graph(input_points)
fij_graph(input_points, commodities)
individual_fij_graph(input_points, commodities)

#-------------------4th end-------------------

#--------------------5th-------------------
from steiner_solution import adjust_polygon_perimeter, inputs, minimiseCostWithAnnotations
from solution_zub_and_zlb import get_zub_zlb
from subgradient_lambda import update_lambda
from data_store_csv import write_to_csv
import numpy as np
import random
from plots import yij_graph, fij_graph, individual_fij_graph

def get_new_lambda(lambda_n, fij, yij, zub, zlb, commodities, selected_edges, n):
    lambda_new, theta_n, s_lambda = update_lambda(lambda_n, fij, yij, zub, zlb, commodities, selected_edges, n)
    return lambda_new, theta_n, s_lambda

# Example input points (initial polygon)
input_points = [(3,0),(5,0), (5,1), (5,3), (3,3), (1,3), (1,2), (1, 4), (3,4), (4,4), (5,4), (4,5), (2,5), (4,6), (4,7), (3,7), (0,7), (0,6), (0,4)]
adjusted_points, total_perimeter = adjust_polygon_perimeter(input_points)
edges_input = [(0, 1), (1, 2), (2, 3), (3, 4), (5, 6), (5, 7), (7, 18), (8, 9), (9, 10), (10, 3), (9, 11), (11, 12), (11, 13), (13, 14), (14, 15), (15, 16), (16, 17), (17, 18), (1, 0), (2, 1), (3, 2), (4, 3), (6, 5), (7, 5), (18, 7), (9, 8), (10, 9), (3, 10), (11, 9), (12, 11), (13, 11), (14, 13), (15, 14), (16, 15), (17, 16), (18, 17)]
print(len(adjusted_points))
print(len(edges_input))
inputs(adjusted_points,total_perimeter, edges_input) #in deges_input removed (3, 4) (4, 3), (4, 5), (5, 4), (17, 18), (18, 17)

# Convert the points to a list of tuples
optimized_points = [tuple(point) for point in adjusted_points]
print(optimized_points) 

station_cost = 1
steiner_cost = 6
edge_cost = 1

# cost, solution_x, solution_y, selected_edges = minimiseCostWithAnnotations(optimized_points, station_cost, steiner_cost, edge_cost)
# points_for_zlb_zub = [(solution_x[i], solution_y[i]) for i in range(len(solution_x))]

cost = station_cost #this is dummy currently for testing actaully its the cost from the steiner tree solution

points_for_zlb_zub = optimized_points
selected_edges = edges_input

speed = 30
capacity = 1000
alpha = 0.5
beta = 0.5
stop_criteria = 1e-3
n = 0

commodities = {
    0 : (0, 6, 10),  #(source, destination, demand)
}

lambda_k = {(k, i, j) : random.random() for k in commodities for (i, j) in selected_edges}
lambda_n = np.array(list(lambda_k.values()))

data = []
theta_n = 0
s_lambda = 0

while True:
    zub, zlb, fij, yij, y_ub, f_ub = get_zub_zlb(points_for_zlb_zub, station_cost, steiner_cost, edge_cost, lambda_k, speed, capacity, alpha, beta, cost, selected_edges, commodities)
    
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
        # print(f"------------------------------------------------")
        # print(f"Iteration {n}: Lambda before update: {lambda_n}")

        lambda_k, s_lambda_dict, theta_n = get_new_lambda(lambda_k, fij, yij, zub, zlb, commodities, selected_edges, n)
        lambda_n = np.array(list(lambda_k.values()))
        s_lambda = np.array(list(s_lambda_dict.values()))

        #lambda_k = {(k, i, j): lambda_n[idx] for idx, (k, i, j) in enumerate(lambda_k.keys())}
        
        # print(f"Iteration {n}: Lambda after update: {lambda_n}")
        # print(f"------------------------------------------------")

write_to_csv(data)

yij_graph(input_points)
fij_graph(input_points, commodities)
individual_fij_graph(input_points, commodities)

#-------------------5th end-------------------

#-------------------Generating random graph based on n----------------
import networkx as nx
import matplotlib.pyplot as plt

def generate_complete_graph_with_coords(n):
    # Create complete graph
    G = nx.complete_graph(n)
    
    # Get 2D coordinates for each node using spring layout (you can use other layouts too)
    pos = nx.spring_layout(G, seed=42)  # seed for reproducibility
    
    # Extract node coordinates in order of node indices
    nodes = [tuple(pos[i]) for i in range(n)]
    
    # Extract edges as list of tuples
    edges = list(G.edges())
    
    # Draw the graph with node coordinates
    nx.draw(G, pos, with_labels=True, node_color='skyblue', edge_color='gray', node_size=500)
    plt.title(f"Complete Graph with {n} Nodes")
    plt.show()
    
    return nodes, edges

# Example usage
n = 5
nodes, edges = generate_complete_graph_with_coords(n)

# Print result
print("Nodes (coordinates):")
print(nodes)

print("\nEdges (index-based):")
print(edges)

#-------------------Generating random graph based on n end----------------
