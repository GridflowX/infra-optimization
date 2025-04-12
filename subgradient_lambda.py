import numpy as np

theta_0 = 5  # Initial step-size numerator

def update_lambda(lambda_n, f_ij, y_ij, Z_min_UB, LD_lambda, commodities, arcs, n):
    """
    Update lambda using the sub-gradient method with step-size constraint.

    Parameters:
    lambda_n  : dict, current lambda values indexed by (k, i, j)
    f_ij      : dict, flow solution indexed by (k, i, j)
    y_ij      : dict, binary arc decisions indexed by (i, j)
    Z_min_UB  : float, best known upper bound
    LD_lambda : float, optimal solution to LR at lambda_n
    commodities : list, set of commodities
    arcs      : list, set of arcs (i, j)
    n         : int, iteration number

    Returns:
    Updated lambda values (dict)
    """

    # Compute step size θ^n
    theta_n = theta_0 / np.sqrt(n)  # theta = 5 / sqrt(n)

    # Compute sub-gradient s_lambda and update lambda
    lambda_new = {}

    # Compute denominator ||s(lambda)||^2
    denominator = 0
    s_lambda = {}

    # Ensure step size is within the range 0 ≤ θ_n ≤ 2
    if not (0 <= theta_n <= 2):
        return lambda_n , s_lambda , theta_n

    for k in commodities:
        for (i, j) in arcs:
            s_lambda[(k, i, j)] = f_ij.get((k, i, j), 0) - y_ij.get((i, j), 0)
            denominator += s_lambda[(k, i, j)] ** 2

    # Prevent division by zero
    if denominator == 0:
        print("Warning: Zero denominator in lambda update. Keeping lambda unchanged.")
        return lambda_n, s_lambda, theta_n

    # Update lambda for each (k, i, j)
    for k in commodities:
        for (i, j) in arcs:
            lambda_new[(k, i, j)] = max(
                lambda_n.get((k, i, j), 0) + (theta_n * ((Z_min_UB - LD_lambda) / denominator) * s_lambda[(k, i, j)]),
                0  # Ensure non-negativity
            )

    # print(f"\n------------------ in update_lambda function ----------------------")
    # print(f"Lambda_n: {lambda_n}")
    # print(f"s_lambda: {s_lambda}")
    # print(f"theta_n: {theta_n}")
    # print(f"Denominator: {denominator}")
    # print(f"Lambda_new: {lambda_new}")
    # print("---------------------------------------------------------------------\n")

    return lambda_new, s_lambda, theta_n
