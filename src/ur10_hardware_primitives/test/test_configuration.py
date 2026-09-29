"""Render and validate hardware models/configuration without opening devices."""
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory as share
from ur10_hardware_primitives.configuration import description, load_cell, merge, moveit_parameters

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture
def measured_cell(tmp_path):
    # TEST FIXTURE ONLY. These values are not a calibration for a physical robot.
    cell = yaml.safe_load((PACKAGE / 'config/cell.yaml').read_text())
    cell.update(configured=True, robot_ip='127.0.0.1', serial_port='/dev/nonexistent-test-device',
        kinematics_file=str(Path(share('ur_description')) / 'config/ur10/default_kinematics.yaml'),
        mount_dimensions=[.06, .06, .02], mount_xyz=[0., 0., .01],
        tcp_xyz=[0., 0., .20], object_dimensions=[.04, .06, .03],
        obstacles=[dict(id='test_table', dimensions=[1., 1., .05], xyz=[.5, .0, -.1], rpy=[0., 0., 0.])])
    path = tmp_path / 'cell.yaml'; path.write_text(yaml.safe_dump(cell))
    return path, cell


def test_default_cell_cannot_start_hardware():
    with pytest.raises(ValueError, match='configured'):
        load_cell(PACKAGE / 'config/cell.yaml')


@pytest.mark.parametrize('key,value', [
    ('tcp_xyz', [float('nan'), 0, 0]), ('object_dimensions', [0, 1, 1]),
    ('serial_port', ''), ('kinematics_file', '/not/a/file'),
    ('gripper_force_multiplier', 2), ('obstacles', [])])
def test_incomplete_measurements_are_rejected(measured_cell, key, value):
    path, cell = measured_cell
    cell[key] = value; path.write_text(yaml.safe_dump(cell))
    with pytest.raises(ValueError): load_cell(path)


def test_combined_model_and_moveit_are_consistent(measured_cell):
    path, _ = measured_cell
    cell = load_cell(path)
    model = description(path, cell)
    robot = ET.fromstring(model)
    assert robot.find('gazebo') is None
    plugins = [p.text for p in robot.findall('ros2_control/hardware/plugin')]
    assert any('URPositionHardwareInterface' in p for p in plugins)
    assert 'robotiq_driver/RobotiqGripperHardwareInterface' in plugins
    assert robot.find("joint[@name='grasp_tcp_joint']/origin").attrib['xyz'] == '0.0 0.0 0.2'
    assert robot.find("link[@name='robotiq_140_base_link']") is not None
    parameters = moveit_parameters(model, PACKAGE / 'config/moveit_controllers.yaml')
    semantic = ET.fromstring(parameters['robot_description_semantic'])
    assert semantic.attrib['name'] == robot.attrib['name']
    assert semantic.find("group[@name='ur_manipulator']/chain").attrib['tip_link'] == 'grasp_tcp'
    names = {n.attrib['name'] for n in robot.findall('link')}
    for pair in semantic.findall('disable_collisions'):
        assert pair.attrib['link1'] in names and pair.attrib['link2'] in names
    assert parameters['moveit_simple_controller_manager']['controller_names'] == ['scaled_joint_trajectory_controller']
    assert not parameters['trajectory_execution']['execution_duration_monitoring']


def test_controller_overlay_preserves_ur_status_controllers():
    base = yaml.safe_load((Path(share('ur_robot_driver')) / 'config/ur_controllers.yaml').read_text())
    override = yaml.safe_load((PACKAGE / 'config/controllers.yaml').read_text())
    merged = merge(base, override)
    manager = merged['controller_manager']['ros__parameters']
    assert manager['update_rate'] == 125
    assert manager['speed_scaling_state_broadcaster'] == base['controller_manager']['ros__parameters']['speed_scaling_state_broadcaster']
    assert manager['robotiq_gripper_controller']['type'] == 'position_controllers/GripperActionController'
    assert merged['scaled_joint_trajectory_controller']['ros__parameters']['speed_scaling_interface_name'] == 'speed_scaling/speed_scaling_factor'


def test_hardware_launch_expands_official_driver_without_starting_it(measured_cell):
    import importlib.util
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.actions import Node

    def module(path, name):
        spec = importlib.util.spec_from_file_location(name, path)
        result = importlib.util.module_from_spec(spec); spec.loader.exec_module(result)
        return result

    path, _ = measured_cell
    context = LaunchContext()
    context.launch_configurations.update(cell_config=str(path), launch_rviz='false')
    launch = module(PACKAGE / 'launch/hardware.launch.py', 'hardware_launch_test')
    actions = launch.start(context)
    for action in actions:
        if isinstance(action, Node):
            action._perform_substitutions(context)
        if isinstance(action, IncludeLaunchDescription):
            for key, value in action.launch_arguments:
                context.launch_configurations[key] = perform_substitutions(context, normalize_to_list_of_substitutions(value))
            driver = module(Path(share('ur_robot_driver')) / 'launch/ur_control.launch.py', 'ur_control_launch_test')
            for arg in driver.generate_launch_description().entities:
                if isinstance(arg, DeclareLaunchArgument):
                    arg.execute(context)
            for node in driver.launch_setup(context):
                if isinstance(node, Node):
                    node._perform_substitutions(context)
    assert context.launch_configurations['initial_joint_controller'] == 'scaled_joint_trajectory_controller'
    assert context.launch_configurations['ur_type'] == 'ur10'
