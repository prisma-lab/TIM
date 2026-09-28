"""Verify live ROS parameter updates, including rclpy's array representation."""
import importlib.util
from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter


spec = importlib.util.spec_from_file_location(
    'target_publisher', Path(__file__).parents[1] / 'scripts/target_publisher.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.fixture
def node():
    rclpy.init()
    node = module.Targets()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_live_position_update_accepts_ros_parameter_message(node):
    # Wire deserialization yields array.array, unlike a Python literal list.
    update = Parameter.from_parameter_msg(
        Parameter('place_position', value=[.5, -.35, .835]).to_parameter_msg())
    assert node.set_parameters([update])[0].successful
    assert list(node.get_parameter('place_position').value) == [.5, -.35, .835]
    node.publish()


@pytest.mark.parametrize('name,value', [
    ('pick_position', [.5, .1]),
    ('place_position', [.5, float('nan'), .835]),
    ('orientation_xyzw', [0., 0., 0., 0.]),
    ('frame_id', ''),
])
def test_invalid_update_preserves_previous_parameter(node, name, value):
    previous = node.get_parameter(name).value
    assert not node.set_parameters([Parameter(name, value=value)])[0].successful
    assert node.get_parameter(name).value == previous
