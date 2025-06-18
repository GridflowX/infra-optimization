from solution_1a_Zub import solution_1a
from solution_Zlb import Solution_Zlb

def get_zub_zlb(points, stc, stic, ec, lambda_n, speed, capacity, alpha, beta, connected_edges, commodities):
    zlb, fij, yij, y_feasible = Solution_Zlb(points, connected_edges, stc, stic, ec, lambda_n, speed, capacity, alpha, beta, commodities)
    zub, y_ub, f_ub = solution_1a(points, stc, stic, ec, connected_edges, speed, capacity, alpha, beta, commodities, y_feasible)

    return zub, zlb, fij, yij, y_ub, f_ub