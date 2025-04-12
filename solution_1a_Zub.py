import pulp
import random

def solution_1a(points, stc, stic, ec,  result_of_cost, connected_edges, speed, capacity, alpha, beta, commodities, y_feasible):
  print(f"\n----------------------in solution 1a ZUB----------------\n")
  # Define nodes and arcs
  nodes = [i for i in range(len(points))]
  arcs = connected_edges
  m1 = (2*len(nodes))-2
  m2 = len(arcs)
  # print(arcs)

  station_cost = stc
  steiner_cost = stic
  edge_cost = ec

  # Parameters
  time = {(i, j): ((((points[i][0] - points[j][0])**2) + ((points[i][1] - points[j][1])**2))**0.5)/speed for (i, j) in arcs}  # t_ij
  cost = {(i, j): edge_cost * ((((points[i][0] - points[j][0])**2) + ((points[i][1] - points[j][1])**2))**0.5) for (i, j) in arcs}
  capacity = {arc: capacity for arc in arcs} # u_ij

  # print(f"nodes : {nodes}")
  # print(f"arcs : {arcs}")
  # print(f"time : {time}")
  # print(f"cost : {cost}")
  # print(f"demand : {demand}")
  # print(f"total_cost : {result_of_cost}")
  # print(f"alpha : {alpha}")
  # print(f"beta : {beta}")

  print(f"----------y_feasible-----------{y_feasible}\n-----------")

  # Define the LP problem
  lp = pulp.LpProblem("Flow_Optimization", pulp.LpMinimize)

  # Variables: flow on each arc
  flow = pulp.LpVariable.dicts("Flow", [(k, i, j) for k in commodities for (i, j) in arcs], lowBound=0, cat="Continuous")

  # Binary decision variables constraint 1g
  y = y_feasible.copy()

  # Objective function: Minimize cost
  lp += (alpha * pulp.lpSum(cost[i, j] * y[i, j] for (i, j) in arcs)) + (beta * pulp.lpSum(flow[k, i, j] * time[i, j] for k in commodities for (i, j) in arcs))

  # constraint 1b
  for k in commodities:
    src, sink, demand_k = commodities[k]
    for node in nodes:
      if node == src:
          lp += (
              pulp.lpSum(flow[k, i, j] for (i, j) in arcs if i == node)
              - pulp.lpSum(flow[k, j, i] for (j, i) in arcs if i == node)
              == demand_k
        )
      elif node == sink:
          lp += (
              pulp.lpSum(flow[k, i, j] for (i, j) in arcs if i == node)
              - pulp.lpSum(flow[k, j, i] for (j, i) in arcs if i == node)
              == -demand_k
          )
      else:
          lp += (
              pulp.lpSum(flow[k, i, j] for (i, j) in arcs if i == node )
              - pulp.lpSum(flow[k, j, i] for (j, i) in arcs if i == node)
              == 0
          )

  # constraint 1c
  for k in commodities:
    _, _, demand_k = commodities[k]
    for (i, j) in arcs:
      lp += flow[k, i, j] <= y[i, j] * demand_k

  #constraint 1d
  for (i, j) in arcs:
    lp += pulp.lpSum(flow[k, i, j] for k  in commodities) <= capacity[i, j]

  # # Constraint 1e
  # lp += pulp.lpSum(y[i, j] for (i, j) in arcs) >= m1  # Lower bound
  # lp += pulp.lpSum(y[i, j] for (i, j) in arcs) <= m2  # Upper bound

  # constraint 1f
  for k in commodities:
    for (i, j) in arcs:
      lp += flow[k, i, j] >= 0

  # Solve the problem
  lp.solve()
  status = lp.solve()
  print("Status:", pulp.LpStatus[lp.status])

  # Debug if infeasible
  if lp.status == -1:  # Infeasible
    print("\nDebugging Constraints:")

    print("\nFlow Conservation Constraints:")
    for k in commodities:
        src, sink, demand_k = commodities[k]
        for node in nodes:
            inflow = pulp.lpSum(flow[k, j, i].varValue for (j, i) in arcs if flow[k, j, i].varValue is not None and i == node)
            outflow = pulp.lpSum(flow[k, i, j].varValue for (i, j) in arcs if flow[k, i, j].varValue is not None and i == node)
            balance = outflow - inflow  # Flow conservation equation

            if node == src:
                print(f"Node {node} (Source): Outflow - Inflow = {balance} (should equal {demand_k})")
            elif node == sink:
                print(f"Node {node} (Sink): Outflow - Inflow = {balance} (should equal {-demand_k})")
            else:
                print(f"Node {node}: Outflow - Inflow = {balance} (should equal 0)")

    # Capacity constraints
    print("\nCapacity Constraints:")
    for (i, j) in arcs:
        lhs = pulp.lpSum(flow[k, i, j].varValue for k in commodities if flow[k, i, j].varValue is not None)
        rhs = capacity[i, j]
        
        if lhs is not None:  # Only check if the variable has a value
            print(f"Arc ({i}, {j}): Flow = {lhs}, Capacity = {rhs} (Flow <= Capacity)")
        else:
            print(f"Arc ({i}, {j}): No value assigned (possible issue)")

    print("\n-----------------------------------------------------------\n")
    
  else:
      # Output results if feasible
      print("Objective Value:", pulp.value(lp.objective))
      for k in commodities:
        print(f"----------------------------Commodity {k}:-----------------------------")
        for (i, j) in arcs:
          print(f"Flow on arc ({i}, {j}):", flow[k, i, j].varValue)
        print("------------------------------------------------------------------------\n")

      for (i, j) in arcs:
        print(f"y({i}, {j}):", y[i, j])
      
      print("\n-----------------------------------------------------------\n")

      return pulp.value(lp.objective), {(i, j): y[i, j] for (i, j) in arcs}, {(k, i, j): flow[k, i, j].varValue for (i, j) in arcs for k in commodities}