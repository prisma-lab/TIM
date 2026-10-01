"""Validate the numeric source used by the parameterized experiment."""
import importlib.util
from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('two_object_targets', PACKAGE / 'scripts/two_object_targets.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.fixture
def targets():
    rclpy.init(args=['--ros-args', '--params-file', str(PACKAGE / 'config/two_object_targets.yaml')])
    node = module.Targets()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_four_distinct_location_topics(targets):
    assert set(targets.publishers_by_location) == {'red_pick', 'red_place', 'blue_pick', 'blue_place'}
    assert {pub.topic_name for pub in targets.publishers_by_location.values()} == {
        '/ur10/two_objects/targets/' + name for name in targets.names}
    targets.publish()


@pytest.mark.parametrize('value', [[1., 2., 3.], [0., 0., float('nan'), 1., 0., 0., 0.],
                                   [0., 0., .8, 0., 0., 0., 0.]])
def test_invalid_pose_update_is_rejected(targets, value):
    before = targets.get_parameter('blue_pick').value
    result = targets.set_parameters([Parameter('blue_pick', value=value)])[0]
    assert not result.successful
    assert targets.get_parameter('blue_pick').value == before


def test_one_location_can_change_without_changing_other_targets(targets):
    red = targets.get_parameter('red_pick').value
    pose = [.5, -.3, .825, 1., 0., 0., 0.]
    assert targets.set_parameters([Parameter('blue_pick', value=pose)])[0].successful
    assert list(targets.get_parameter('blue_pick').value) == pose
    assert targets.get_parameter('red_pick').value == red
