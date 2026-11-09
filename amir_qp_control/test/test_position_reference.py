"""A stalled motor must get a growing, bounded target, with no queued reversal."""
import sys
from pathlib import Path
from types import SimpleNamespace, MethodType
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'launch'))
from position_reference import PositionReference, cbf_command_compatible
from qp_tracking_controller import QPTrackingController


def advance(reference, measured, velocity, accumulate=True, q_min=None, q_max=None):
    return reference.advance(np.array(measured), np.array(velocity), .02,
                             np.full(4, -2.) if q_min is None else np.array(q_min),
                             np.full(4, 2.) if q_max is None else np.array(q_max), accumulate)


def test_stalled_joint_target_grows_to_cap_and_keeps_motor_speed():
    reference = PositionReference(.02)
    first, _ = advance(reference, [0.] * 4, [.18, 0., 0., 0.])
    second, _ = advance(reference, [0.] * 4, [.18, 0., 0., 0.])
    assert first[0] == pytest.approx(.0036)
    assert second[0] == pytest.approx(.0072)
    for _ in range(100):
        target, speed = advance(reference, [0.] * 4, [.18, 0., 0., 0.])
    assert target[0] == pytest.approx(.02)
    assert speed[0] == pytest.approx(.18)
    # Once feedback catches up, the target can keep progressing.
    target, _ = advance(reference, [.01, 0., 0., 0.], [.18, 0., 0., 0.])
    assert target[0] == pytest.approx(.0236)


def test_reversal_and_zero_velocity_discard_previous_lead_per_joint():
    reference = PositionReference(.02)
    for _ in range(10):
        advance(reference, [0.] * 4, [.18, -.18, .18, .18])
    target, speed = advance(reference, [0.] * 4, [-.18, .18, 0., .18])
    np.testing.assert_allclose(target, [-.0036, .0036, 0., .02])
    np.testing.assert_allclose(speed, [-.18, .18, 0., .18])


def test_explicit_reset_discards_backlog_and_uses_measured_step():
    reference = PositionReference(.02)
    for _ in range(10):
        advance(reference, [0.] * 4, [.18] * 4)
    target, speed = advance(reference, [0.] * 4, [.18] * 4, accumulate=False)
    np.testing.assert_allclose(target, [.0036] * 4)
    np.testing.assert_allclose(speed, [.18] * 4)


def test_reset_and_absolute_joint_limit():
    reference = PositionReference(.02)
    for _ in range(10):
        advance(reference, [0.] * 4, [.18] * 4)
    reference.reset()
    target, _ = advance(reference, [0.] * 4, [.18] * 4)
    np.testing.assert_allclose(target, [.0036] * 4)
    target, speed = advance(reference, [1.999, 0., 0., 0.], [.18, 0., 0., 0.])
    assert target[0] == pytest.approx(2.)
    assert speed[0] == pytest.approx(.05)


def test_unreachable_lead_interval_holds_instead_of_jumping_to_joint_limit():
    reference = PositionReference(.02)
    target, speed = advance(reference, [2.1, 0., 0., 0.], [-.18, 0., 0., 0.])
    np.testing.assert_allclose(target, [2.1, 0., 0., 0.])
    np.testing.assert_allclose(speed, [0.] * 4)


@pytest.mark.parametrize('lead', [0., -.02, float('nan'), float('inf')])
def test_invalid_lead_is_rejected(lead):
    with pytest.raises(ValueError):
        PositionReference(lead)


def make_control_node(active_cbf=False):
    node = SimpleNamespace(
        qp_enabled=True, current_q=np.zeros(4), controller_stopped=False,
        position_reference=PositionReference(.02), position_command_mode="integrated",
        publish_stop_state=Mock(), publish_fk_pose=Mock(), publish_cbf_markers=Mock(),
        get_obstacles_in_kinematics_frame=lambda: (np.empty((0, 3)), np.empty(0)),
        target_pose=object(), get_target_in_kinematics_frame=lambda: np.ones(3),
        solve_qp=lambda *args: (np.array([.18, 0., 0., 0.]), .2),
        active_cbf_constraint_count=int(active_cbf),
        cbf_constraint_matrix=np.array([[1., 0., 0., 0.]]) if active_cbf else np.empty((0, 4)),
        cbf_constraint_lower_bound=np.array([-.1]) if active_cbf else np.empty(0),
        cbf_alpha=5., get_logger=lambda: Mock(),
        dt=.02, q_min=np.full(4, -2.), q_max=np.full(4, 2.),
        position_tolerance=.005, stop_after_reaching_target=False,
        target_reached_cycle_count=0, frequency=50.,
        joint5_hold_position=.5, command_pub=Mock(), velocity_command_pub=Mock(),
    )
    node.publish_command = MethodType(QPTrackingController.publish_command, node)
    return node


@pytest.mark.parametrize('active_cbf', [False, True])
def test_control_loop_integrates_with_or_without_cbf_and_hold_clears_reference(active_cbf):
    node = make_control_node(active_cbf)
    for _ in range(10):
        QPTrackingController.control_callback(node)
    assert node.command_pub.publish.call_args.args[0].data[0] == pytest.approx(.02)
    assert node.velocity_command_pub.publish.call_args.args[0].data[0] == pytest.approx(.18)
    node.target_pose = None
    QPTrackingController.control_callback(node)
    np.testing.assert_allclose(node.command_pub.publish.call_args.args[0].data[:4], np.zeros(4))
    np.testing.assert_allclose(node.velocity_command_pub.publish.call_args.args[0].data, np.zeros(5))
    assert node.position_reference.reference is None


def test_old_lead_conflicting_with_new_cbf_is_replaced_then_integration_resumes():
    node = make_control_node()
    for _ in range(10):
        QPTrackingController.control_callback(node)
    node.active_cbf_constraint_count = 1
    node.cbf_constraint_matrix = np.array([[-1., 1., 0., 0.]])
    node.cbf_constraint_lower_bound = np.array([-.005])
    node.solve_qp = lambda *args: (np.array([.1, .2, 0., 0.]), .2)
    QPTrackingController.control_callback(node)
    np.testing.assert_allclose(node.command_pub.publish.call_args.args[0].data[:4], [.002, .004, 0., 0.])
    QPTrackingController.control_callback(node)
    np.testing.assert_allclose(node.command_pub.publish.call_args.args[0].data[:4], [.004, .008, 0., 0.])


def test_cbf_check_rejects_incompatible_velocity_or_accumulated_endpoint():
    matrix = np.array([[-1., 1., 0., 0.]])
    bound = np.array([-.005])
    # Safe velocity but an old lead pointing through the linearized barrier.
    assert not cbf_command_compatible(np.zeros(4), [.02, .004, 0., 0.],
                                      [.1, .2, 0., 0.], .02, matrix, bound, 5.)
    # A safe endpoint alone is insufficient if the sent velocity violates CBF.
    assert not cbf_command_compatible(np.zeros(4), [.002, .004, 0., 0.],
                                      [.2, .1, 0., 0.], .02, matrix, bound, 5.)


def test_cbf_check_permits_one_step_recovery_from_negative_h():
    assert cbf_command_compatible(np.zeros(4), [.0002, 0., 0., 0.],
                                   [.01, 0., 0., 0.], .02,
                                   np.array([[1., 0., 0., 0.]]), np.array([.01]), 5.)


def test_incompatible_fresh_cbf_command_sends_hold_and_clears_reference():
    node = make_control_node(True)
    node.cbf_constraint_matrix = np.array([[-1., 0., 0., 0.]])
    node.cbf_constraint_lower_bound = np.array([0.])
    QPTrackingController.control_callback(node)
    np.testing.assert_allclose(node.velocity_command_pub.publish.call_args.args[0].data, np.zeros(5))
    assert node.position_reference.reference is None
