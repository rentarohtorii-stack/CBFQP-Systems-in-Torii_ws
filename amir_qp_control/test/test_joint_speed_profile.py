"""Configured magnitudes must not reverse motion or override QP constraints."""
import sys
from pathlib import Path

import cvxpy as cp
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'launch'))
from joint_speed_profile import project_joint_speeds, validate_joint_speeds


def test_speed_config_is_accepted_by_ros_parameter_file_parser():
    from rclpy.utilities import remove_ros_args
    config = Path(__file__).resolve().parents[1] / 'config' / 'joint_speeds.yaml'
    assert remove_ros_args(args=['qp_check', '--ros-args', '--params-file', str(config)]) == ['qp_check']


def test_speed_magnitudes_take_direction_and_stop_from_qp():
    velocity = cp.Variable(4)
    result = project_joint_speeds([.07, -.12, 0., .02], [.1, .15, .2, .05], velocity, [])
    np.testing.assert_allclose(result, [.1, -.15, 0., .05], atol=1e-6)


def test_cbf_coupling_adjusts_speeds_instead_of_overwriting_constraint():
    velocity = cp.Variable(4)
    result = project_joint_speeds([.2, .1, 0., 0.], [.2, .2, .2, .2], velocity,
                                  [velocity[0] - velocity[1] >= .1])
    assert result is not None
    assert result[0] - result[1] >= .1 - 1e-6
    assert np.all(result[:2] >= -1e-6)
    np.testing.assert_allclose(result, [.2, .1, 0., 0.], atol=1e-6)


def test_global_joint_and_next_position_limits_are_kept():
    velocity = cp.Variable(4)
    result = project_joint_speeds([.1, -.1, .01, .01], [.4, .4, .4, 0.], velocity,
                                  [velocity <= .3, velocity >= -.3,
                                   .99 + .02 * velocity[0] <= .991])
    np.testing.assert_allclose(result, [.05, -.3, .3, 0.], atol=1e-6)


def test_infeasible_profile_does_not_emit_an_unchecked_velocity():
    velocity = cp.Variable(4)
    assert project_joint_speeds([.4, 0., 0., 0.], [.2] * 4, velocity,
                                [velocity[0] >= .3]) is None


@pytest.mark.parametrize('speeds', [[-.1, .1, .1, .1], [.1, np.nan, .1, .1],
                                    [.1, .1, np.inf, .1], [.1] * 5])
def test_invalid_profiles_are_rejected(speeds):
    with pytest.raises(ValueError):
        validate_joint_speeds(speeds)
