"""Bounded integration of joint targets, with no queued motion after stopping."""
import numpy as np


def cbf_command_compatible(measured, target, velocity, dt, matrix, lower_bound, alpha):
    """Check current CBF velocity bounds and the accumulated linearized endpoint.

    For a safe pair, require h + A*(target-measured) >= 0. If h is already
    negative, permit the original one-step recovery instead of demanding an
    instantaneous jump back into the safe set. This is a local model check,
    not a proof of collision-free tracking by the physical motor controller.
    """
    if len(lower_bound) == 0:
        return True
    lead = np.asarray(target) - np.asarray(measured)
    endpoint_bound = np.where(lower_bound <= 0., lower_bound / alpha, dt * lower_bound)
    return bool(
        np.all(matrix @ velocity >= lower_bound - 1e-5)
        and np.all(matrix @ lead >= endpoint_bound - 1e-5)
    )


class PositionReference:
    def __init__(self, max_lead):
        if not np.isfinite(max_lead) or max_lead <= 0:
            raise ValueError("max_position_leadは正の有限値[rad]にしてください。")
        self.max_lead = float(max_lead)
        self.reset()

    def reset(self):
        self.reference = None

    def advance(self, measured, velocity, dt, q_min, q_max, accumulate=True):
        measured = np.asarray(measured, dtype=float)
        velocity = np.asarray(velocity, dtype=float)
        lower = np.maximum(q_min, measured - self.max_lead)
        upper = np.minimum(q_max, measured + self.max_lead)
        if np.any(lower > upper):
            # Cannot satisfy both absolute joint limits and the lead bound.
            self.reset()
            return measured.copy(), np.zeros_like(velocity)

        base = measured.copy() if self.reference is None or not accumulate else self.reference.copy()
        # Discard a previous lead when the requested direction reverses, or when
        # measurement has overtaken it. Never drive toward an obsolete target.
        keep_lead = (base - measured) * velocity > 0.
        base = np.where(keep_lead, base, measured)
        base = np.clip(base, lower, upper)
        target = np.clip(base + dt * velocity, lower, upper)

        moving = np.abs(velocity) > 1e-9
        target = np.where(moving, target, measured)
        distance = target - measured
        toward_target = distance * velocity > 0.
        # At the lead cap, retain a nonzero speed while the motor catches up.
        # Bound it by both QP speed and the remaining position distance per tick.
        sent_velocity = np.where(
            toward_target, np.sign(velocity) * np.minimum(np.abs(velocity), np.abs(distance) / dt), 0.)
        self.reference = target.copy()
        return target, sent_velocity
