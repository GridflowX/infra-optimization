from solution_1a_Zub import solution_1a
from solution_Zlb import Solution_Zlb
from steiner_solution import minimiseCostWithAnnotations

def get_zub_zlb(points, stc, stic, ec, lambda_n, speed, capacity, alpha, beta, result_of_cost, connected_edges, commodities):
    # print(points)
    zlb, fij, yij, y_feasible = Solution_Zlb(points, connected_edges, stc, stic, ec, lambda_n, speed, capacity, alpha, beta, commodities)
    zub, y_ub, f_ub = solution_1a(points, stc, stic, ec, result_of_cost, connected_edges, speed, capacity, alpha, beta, commodities, y_feasible)

    return zub, zlb, fij, yij, y_ub, f_ub