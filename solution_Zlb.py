import numpy as np
import matplotlib.pyplot as plt
import random
import pulp

def LR_1(points, selected_edges, stc, stic, ec, lamda, alphaa, commodities):
  print("\n---------------------------------LR1 Print statements------------------------")
  nodes = [i for i in range(len(points))]
  arcs = selected_edges
  lambda_k = lamda
  alpha = alphaa
  m1 = (2*(len(nodes)))-2
  m2 = len(arcs)

  # print("nodes:",nodes)
  # print("arcs:",arcs)
  # print("lambda list:", lamda)
  # print("lambdas:",lambda_k)
  # print("alpha:",alpha)
  print("m1:",m1)
  print("m2:",m2)

  # Costs
  station_cost = stc  # in lakhs
  steiner_cost = stic  # in lakhs
  edge_cost_per_unit = ec  # in lakhs per km
  stations_with_steiner_cost = (station_cost * (len(nodes)-1)) + steiner_cost

  cost = {(i, j): edge_cost_per_unit * ((((points[i][0] - points[j][0])**2) + ((points[i][1] - points[j][1])**2))**0.5) for (i,j) in arcs}

  # print("cost:",cost)
  # print()

  # Define the LP problem
  lp = pulp.LpProblem("LR1_Optimization", pulp.LpMinimize)

  # Binary decision variables constraint 6c
  y = pulp.LpVariable.dicts("y", arcs, cat="Binary")

  # Objective function: Minimize total cost 6a
  lp += pulp.lpSum((alpha * cost[i, j] - pulp.lpSum(lambda_k[k, i, j] for k in commodities)) * y[i, j] for (i, j) in arcs)

  # Constraints 6b
  lp += pulp.lpSum(y[i, j] for (i, j) in arcs) >= m1  # Lower bound
  lp += pulp.lpSum(y[i, j] for (i, j) in arcs) <= m2  # Upper bound

  # # Print the Objective Function
  # print("Objective Function:")
  # print(lp.objective)  # Inbuilt method
  # print("-" * 40)

  # # Print All Constraints
  # print("Constraints:")
  # for name, constraint in lp.constraints.items():
  #     print(f"{name}: {constraint}")
  # print("-" * 40)

  # Solve the problem
  lp.solve()

  # Output the results
  print("Status:", pulp.LpStatus[lp.status])
  print("Objective Value:", pulp.value(lp.objective))
  for (i, j) in arcs:
      print(f"y({i}, {j}):", y[i, j].varValue)

  print("-----------------------------------------------------------------------------\n")
  
  return pulp.value(lp.objective), {(i, j) : y[i, j].varValue for (i, j) in arcs}

from pulp import LpProblem, LpMinimize, LpVariable, lpSum

def LR_2(points, selected_edges, lamda, beeta, speed, capacity, commodities):
    print("\n---------------------------------LR2 Print statements------------------------")
    # Define the parameters
    nodes = [i for i in range(len(points))]
    arcs = selected_edges
    lambda_k = lamda
    beta = beeta

    t_ij = {(i, j): ((((points[i][0] - points[j][0])**2) + ((points[i][1] - points[j][1])**2))**0.5)/speed for (i, j) in arcs}  # t_ij
    u_ij = capacity # u_ij

    # print("nodes:",nodes)
    # print("arcs:",arcs)
    # print("lambda list:", lamda)
    # print("lambdas:",lambda_k)
    # print("beta:",beta)
    # print("time:",t_ij)
    # print("capacity:",u_ij)

    # Create problem
    prob = LpProblem("Flow_Optimization", LpMinimize)

    # Define decision variables  constraint 7d
    f = pulp.LpVariable.dicts("Flow", [(k, i, j) for (i, j) in arcs for k in commodities], lowBound = 0, upBound = 1, cat = "Continuous")

    # Objective function  7a
    prob += lpSum((beta * t_ij[i, j] * demand_k + lambda_k[k, i, j]) * f[k, i, j] for k, (_, _, demand_k) in commodities.items() for (i, j) in arcs)

    # constraint 7b
    for node in nodes:
      for k in commodities:
        src, sink, _ = commodities[k]
        if node == src:
            prob += (
              pulp.lpSum(f[k, i, j] for (i, j) in arcs if i == node)
              - pulp.lpSum(f[k, j, i] for (j, i) in arcs if i == node)
              == 1
          )
        elif node == sink:
            prob += (
              pulp.lpSum(f[k, i, j] for (i, j) in arcs if i == node)
              - pulp.lpSum(f[k, j, i] for (j, i) in arcs if i == node)
              == -1
          )
        else:
            prob += (
              pulp.lpSum(f[k, i, j] for (i, j) in arcs if i == node )
              - pulp.lpSum(f[k, j, i] for (j, i) in arcs if i == node)
              == 0
          )

    # constraints 7c
    for (i, j) in arcs:
      prob += pulp.lpSum(f[k, i, j] * commodities[k][2] for k in commodities) <= u_ij[i, j]

    # constraint 7d
    for (i, j) in arcs:
      for k in commodities:
        prob += f[k, i, j] >= 0
        prob += f[k, i, j] <= 1

    # Solve problem
    prob.solve()

    print("Solver Status:", pulp.LpStatus[prob.status])
    # print(prob)

    # Debug if infeasible
    if prob.status == -1:  # Infeasible
        print("\nDebugging Constraints:")

        # Flow conservation constraints
        print("\nFlow Conservation Constraints:")
        for k in commodities:
          src, sink, demand_k = commodities[k]
          for node in nodes:
            inflow = pulp.lpSum(f[k, j, i].varValue for (j, i) in arcs if f[k, j, i].varValue is not None and i == node)
            outflow = pulp.lpSum(f[k, i, j].varValue for (i, j) in arcs if f[k, i, j].varValue is not None and i == node)
            balance = outflow - inflow  # Flow conservation equation

            if node == src:
                print(f"Node {node} (Source): Outflow - Inflow = {balance} (should equal 1)")
            elif node == sink:
                print(f"Node {node} (Sink): Outflow - Inflow = {balance} (should equal -1)")
            else:
                print(f"Node {node}: Outflow - Inflow = {balance} (should equal 0)")

        # Capacity constraints
        print("\nCapacity Constraints:")
        for (i, j) in arcs:
            lhs = f[i, j].varValue
            rhs = u_ij[i, j]
            if lhs is not None:  # Only check if the variable has a value
                print(f"Arc ({i}, {j}): Flow = {lhs}, Capacity = {rhs} (Flow <= Capacity)")
            else:
                print(f"Arc ({i}, {j}): No value assigned (possible issue)")

        # Variable bounds
        print("\nVariable Bounds:")
        for (i, j) in arcs:
          value = pulp.lpSum(f[k, i, j].varValue for k in commodities if f[k, i, j].varValue is not None)
        
          if value is not None:  # Only check if the variable has a value
            if value < 0 or value > max(demand for (_, _, demand) in commodities.values()):
                print(f"Variable Flow[{i}, {j}] = {value} (out of bounds [0, max demand])")
            else:
                print(f"Variable Flow[{i}, {j}] = {value} (within bounds)")
        else:
            print(f"Variable Flow[{i}, {j}] has no value assigned (possible issue)")
        
        print("-----------------------------------------------------------------------------\n")
      
    else:
        # Output results if feasible
        print("Objective Value:", pulp.value(prob.objective))
        for k in commodities:
          print(f"----------------------------Commodity {k}:-----------------------------")
          for (i, j) in arcs:
            print(f"Flow on arc ({i}, {j}):", f[k, i, j].varValue)
          print("------------------------------------------------------------------------\n")

        print("--------------------------y_feasible------------------------------")
        
        y_feasible = {
            (i, j): int(any(f[k, i, j].varValue > 0 for k in commodities))
            for (i, j) in arcs
        }

        print(y_feasible)
        print("------------------------------------------------------------------------\n")

        print("-----------------------------------------------------------------------------\n")
      
        return pulp.value(prob.objective), {(k, i, j): f[k, i, j].varValue for (k, i, j) in f.keys()}, y_feasible

def Solution_Zlb(points, selected_edges, stc, stic, ec, lambda_n, speed, capacity, alpha, beta, commodities):
  # print(selected_edges)
  lr1_result, yij = LR_1(points, selected_edges, stc, stic, ec, lambda_n, alpha, commodities)
  lr2_result, fij, y_feasible = LR_2(points, selected_edges, lambda_n, beta, speed, capacity, commodities)
  print("LR1 Solution:", lr1_result)
  print("LR2 Solution:", lr2_result)

  result = lr1_result + lr2_result
  print("Total Solution:", result)

  return result, fij, yij, y_feasible