"""Choose speed magnitudes while retaining QP direction and feasibility."""
import cvxpy as cp
import numpy as np


def validate_joint_speeds(values):
    speeds = np.asarray(values, dtype=float)
    if speeds.shape != (4,) or not np.all(np.isfinite(speeds)) or np.any(speeds < 0):
        raise ValueError("joint_speedsはJoint_1～4の4要素、0以上の有限値[rad/s]にしてください。")
    return speeds


def project_joint_speeds(nominal, speeds, velocity, constraints, zero_threshold=1e-6):
    """Return the nearest feasible speed profile, or None if it cannot be met.

    Speeds are desired magnitudes and hard per-joint caps. The original tracking
    solution supplies direction; zero joints stay stopped. The original CBF,
    position and global joint velocity constraints are enforced in this solve.
    """
    speeds = validate_joint_speeds(speeds)
    nominal = np.asarray(nominal, dtype=float)
    direction = np.where(np.abs(nominal) > zero_threshold, np.sign(nominal), 0.)
    desired = direction * speeds
    profile_constraints = list(constraints) + [
        velocity <= speeds,
        velocity >= -speeds,
        cp.multiply(direction, velocity) >= 0.,
    ]
    stopped = np.flatnonzero(direction == 0.)
    if len(stopped):
        profile_constraints.append(velocity[stopped] == 0.)
    problem = cp.Problem(cp.Minimize(cp.sum_squares(velocity - desired)), profile_constraints)
    try:
        problem.solve(solver=cp.OSQP, warm_start=True, verbose=False,
                      eps_abs=1e-7, eps_rel=1e-7)
    except cp.error.SolverError:
        return None
    if problem.status not in {cp.OPTIMAL, cp.OPTIMAL_INACCURATE} or velocity.value is None:
        return None
    result = np.asarray(velocity.value, dtype=float).reshape(4)
    if not np.all(np.isfinite(result)):
        return None
    result[stopped] = 0.0
    velocity.value = result
    # Do not publish a numerically invalid/infeasible result as a valid command.
    if any(
        np.any(np.asarray(constraint.violation()) > 1e-6) for constraint in profile_constraints
    ):
        return None
    return result
