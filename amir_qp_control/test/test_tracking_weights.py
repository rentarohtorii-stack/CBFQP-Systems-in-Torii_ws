"""Exercise axis priorities with coupled motion and the real QP solver."""
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'launch'))
from qp_tracking_controller import QPTrackingController


def solve(weights, target, jacobian, limit=1.0, mode="measured", lead=.02):
    node = SimpleNamespace(
        kinematics=SimpleNamespace(
            forward_kinematics=lambda q: jacobian @ q,
            jacobian=lambda q: jacobian,
        ),
        tracking_weights=np.array(weights), kp=1.0, position_tolerance=1e-6,
        max_cartesian_velocity=10.0, regularization=.01,
        qdot_max=np.full(4, limit), q_min=np.full(4, -10.),
        q_max=np.full(4, 10.), dt=.02, cbf_enabled=False,
        get_logger=lambda: Mock(), joint_speed_mode="qp", position_command_mode=mode,
        max_position_lead=lead,
    )
    qdot, error = QPTrackingController.solve_qp(
        node, np.zeros(4), np.array(target), np.empty((0, 3)), np.empty(0))
    return qdot, error


def test_equal_weights_preserve_unweighted_least_squares():
    jacobian = np.array([[1., 0., 0., .2], [0., 1., 0., .3], [0., 0., 1., .4]])
    target = np.array([.1, -.2, .3])
    qdot, error = solve([1., 1., 1.], target, jacobian)
    expected = np.linalg.solve(jacobian.T @ jacobian + .01 * np.eye(4), jacobian.T @ target)
    np.testing.assert_allclose(qdot, expected, atol=1e-6)
    assert error == pytest.approx(np.linalg.norm(target))


@pytest.mark.parametrize('axis', [0, 1, 2])
def test_larger_axis_weight_reduces_that_axis_residual(axis):
    # All axes share one joint, so conflicting goals require a tradeoff.
    jacobian = np.zeros((3, 4))
    jacobian[:, 0] = 1.
    target = np.full(3, -.1)
    target[axis] = .1
    equal, _ = solve([1., 1., 1.], target, jacobian)
    weights = np.ones(3)
    weights[axis] = 9.
    weighted, _ = solve(weights, target, jacobian)
    assert abs((jacobian @ weighted - target)[axis]) < abs((jacobian @ equal - target)[axis])


def test_high_weight_respects_joint_velocity_limit():
    jacobian = np.eye(3, 4)
    qdot, _ = solve([1., 1., 100.], [0., 0., 1.], jacobian, limit=.2)
    assert qdot[2] == pytest.approx(.2, abs=1e-5)
    assert np.all(np.abs(qdot) <= .2 + 1e-5)


def test_integrated_mode_keeps_single_step_within_lead_bound():
    qdot, _ = solve([1., 1., 1.], [1., 0., 0.], np.eye(3, 4),
                    mode="integrated", lead=.002)
    assert qdot[0] == pytest.approx(.1, abs=1e-5)
    assert np.all(np.abs(.02 * qdot) <= .002 + 1e-6)
