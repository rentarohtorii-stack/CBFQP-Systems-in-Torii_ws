"""OFF must bypass the solver and keep publishing stop/hold without a map."""
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'launch'))
from qp_tracking_controller import QPTrackingController


def test_disabled_continuously_holds_without_map_or_target():
    node = SimpleNamespace(qp_enabled=False, current_q=np.array([.1, .2, .3, .4]),
                           position_reference=Mock(),
                           publish_stop_state=Mock(), publish_command=Mock(),
                           solve_qp=Mock(side_effect=AssertionError('solver called')))
    for _ in range(3):
        QPTrackingController.control_callback(node)
    assert node.publish_stop_state.call_count == 3
    assert node.publish_command.call_count == 3
    for call in node.publish_command.call_args_list:
        np.testing.assert_array_equal(call.args[0], node.current_q)
    node.solve_qp.assert_not_called()


def test_disabled_requests_stop_even_before_first_joint_state():
    node = SimpleNamespace(qp_enabled=False, current_q=None,
                           position_reference=Mock(),
                           publish_stop_state=Mock(), publish_command=Mock())
    QPTrackingController.control_callback(node)
    node.publish_stop_state.assert_called_once()
    node.publish_command.assert_not_called()


@pytest.mark.parametrize('enabled', [False, True])
def test_toggle_clears_old_target_and_keeps_timer_running(enabled):
    node = SimpleNamespace(qp_enabled=not enabled, target_pose=object(),
                           position_reference=Mock(),
                           target_reached_cycle_count=12, controller_stopped=True,
                           timer=Mock(), publish_stop_state=Mock())
    response = SimpleNamespace()
    QPTrackingController.set_enabled_callback(node, SimpleNamespace(data=enabled), response)
    assert node.qp_enabled == enabled
    assert node.target_pose is None
    assert not node.controller_stopped
    assert node.target_reached_cycle_count == 0
    assert response.success
    node.timer.reset.assert_called_once()
    node.publish_stop_state.assert_called_once()
    node.position_reference.reset.assert_called_once()


@pytest.mark.parametrize('enabled', [False, True])
def test_stop_flag_is_inverse_of_enabled(enabled):
    node = SimpleNamespace(qp_enabled=enabled, stop_pub=Mock())
    QPTrackingController.publish_stop_state(node)
    assert node.stop_pub.publish.call_args.args[0].data == (not enabled)


def test_position_command_joint_order_and_joint5_hold():
    node = SimpleNamespace(joint5_hold_position=.5, command_pub=Mock(), velocity_command_pub=Mock(),
                           position_reference=Mock())
    QPTrackingController.publish_command(node, np.array([.1, .2, .3, .4]))
    assert list(node.command_pub.publish.call_args.args[0].data) == [.1, .2, .3, .4, .5]
    assert list(node.velocity_command_pub.publish.call_args.args[0].data) == [0.] * 5
    node.position_reference.reset.assert_called_once()


def test_position_command_waits_for_joint5_feedback():
    node = SimpleNamespace(joint5_hold_position=None, command_pub=Mock(), velocity_command_pub=Mock(),
                           position_reference=Mock())
    QPTrackingController.publish_command(node, np.array([.1, .2, .3, .4]))
    node.command_pub.publish.assert_not_called()
    node.velocity_command_pub.publish.assert_not_called()


def test_selected_speed_is_published_with_joint5_stopped():
    node = SimpleNamespace(joint5_hold_position=.5, command_pub=Mock(), velocity_command_pub=Mock())
    QPTrackingController.publish_command(node, np.array([.1, .2, .3, .4]),
                                         np.array([.1, -.15, .2, 0.]))
    assert list(node.velocity_command_pub.publish.call_args.args[0].data) == [.1, -.15, .2, 0., 0.]
