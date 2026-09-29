"""Build one calibrated description shared by UR control and MoveIt."""

from copy import deepcopy
from itertools import combinations
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def vector(data, key, positive=False):
    values = data.get(key)
    if not isinstance(values, list) or len(values) != 3 or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or
            not math.isfinite(v) or (positive and v <= 0) for v in values):
        raise ValueError(f'{key} must contain three finite {"positive " if positive else ""}numbers')
    return values


def load_cell(path):
    cell = yaml.safe_load(Path(path).read_text())
    if not isinstance(cell, dict) or cell.get('configured') is not True:
        raise ValueError('Fill the measured cell.yaml settings and set configured: true before hardware launch')
    for key in ('robot_ip', 'serial_port', 'kinematics_file'):
        if not isinstance(cell.get(key), str) or not cell[key].strip():
            raise ValueError(f'Configure {key}')
    calibration = Path(cell['kinematics_file']).expanduser()
    if not calibration.is_absolute() or not calibration.is_file():
        raise ValueError('kinematics_file must be an existing absolute path inside this environment')
    cell['kinematics_file'] = str(calibration)
    for key in ('mount_xyz', 'mount_rpy', 'mount_box_xyz', 'tcp_xyz', 'tcp_rpy',
                'object_offset_xyz', 'object_offset_rpy'):
        vector(cell, key)
    for key in ('object_dimensions', 'mount_dimensions'):
        vector(cell, key, positive=True)
    for key in ('gripper_speed_multiplier', 'gripper_force_multiplier'):
        value = cell.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 < value <= 1:
            raise ValueError(f'{key} must be in (0, 1]')
    if not isinstance(cell.get('obstacles'), list) or not cell['obstacles']:
        raise ValueError('Add the measured table/fixtures to obstacles before hardware launch')
    names = set()
    for obstacle in cell['obstacles']:
        if not isinstance(obstacle, dict) or not isinstance(obstacle.get('id'), str) or not obstacle['id']:
            raise ValueError('Each obstacle needs an id')
        if obstacle['id'] in names or obstacle['id'] == 'workpiece':
            raise ValueError('Obstacle ids must be distinct and cannot be workpiece')
        names.add(obstacle['id'])
        vector(obstacle, 'dimensions', positive=True)
        vector(obstacle, 'xyz')
        vector(obstacle, 'rpy')
    return cell


def merge(base, overrides):
    result = deepcopy(base)
    for key, value in overrides.items():
        result[key] = merge(result[key], value) if (
            isinstance(value, dict) and isinstance(result.get(key), dict)) else deepcopy(value)
    return result


def description(cell_path, cell):
    import xacro
    from ament_index_python.packages import get_package_share_directory as share
    ur = Path(share('ur_description'))
    driver = Path(share('ur_robot_driver'))
    package = Path(share('ur10_hardware_primitives'))
    mappings = {
        'name': 'ur10', 'ur_type': 'ur10', 'cell_config': str(Path(cell_path).resolve()),
        'robot_ip': cell['robot_ip'], 'kinematics_params': cell['kinematics_file'],
        'joint_limit_params': str(ur / 'config/ur10/joint_limits.yaml'),
        'physical_params': str(ur / 'config/ur10/physical_parameters.yaml'),
        'visual_params': str(ur / 'config/ur10/visual_parameters.yaml'),
        'script_filename': str(Path(share('ur_client_library')) / 'resources/external_control.urscript'),
        'input_recipe_filename': str(driver / 'resources/rtde_input_recipe.txt'),
        'output_recipe_filename': str(driver / 'resources/rtde_output_recipe.txt'),
        'safety_limits': 'true', 'use_fake_hardware': 'false',
        'use_tool_communication': 'false', 'headless_mode': 'false',
    }
    return xacro.process_file(str(package / 'urdf/ur10_2f140.urdf.xacro'), mappings=mappings).toxml()


def semantics(robot_description):
    import xacro
    from ament_index_python.packages import get_package_share_directory as share
    robot = ET.fromstring(robot_description)
    srdf = ET.fromstring(xacro.process_file(
        str(Path(share('ur_moveit_config')) / 'srdf/ur.srdf.xacro'),
        mappings={'name': 'ur', 'prefix': ''}).toxml())
    srdf.set('name', robot.attrib['name'])
    srdf.find("group[@name='ur_manipulator']/chain").set('tip_link', 'grasp_tcp')
    hand = ET.SubElement(srdf, 'group', name='gripper')
    ET.SubElement(hand, 'joint', name='finger_joint')
    ET.SubElement(srdf, 'end_effector', name='robotiq', parent_link='grasp_tcp',
                  group='gripper', parent_group='ur_manipulator')
    # Retain the official UR policy; extend only for adjacent/fixed geometry.
    links = {node.attrib['name'] for node in robot.findall('link')}
    parent = {name: name for name in links}
    def root(name):
        while parent[name] != name:
            name = parent[name]
        return name
    disabled = {tuple(sorted((n.attrib['link1'], n.attrib['link2'])))
                for n in srdf.findall('disable_collisions')}
    extra = set()
    for joint in robot.findall('joint'):
        a, b = joint.find('parent').attrib['link'], joint.find('child').attrib['link']
        extra.add(tuple(sorted((a, b))))
        if joint.attrib['type'] == 'fixed':
            parent[root(a)] = root(b)
    for a, b in combinations(sorted(links), 2):
        if root(a) == root(b):
            extra.add((a, b))
    # The 2F-140 is a closed linkage represented as a tree. These same-finger
    # parts meet physically but are not parent/child pairs in that tree.
    # Keep opposite fingers, arm, mount, and external contacts checked.
    for side in ('left', 'right'):
        extra.add(tuple(sorted((side + '_inner_finger', side + '_inner_knuckle'))))
        extra.add(tuple(sorted((side + '_inner_knuckle', side + '_outer_knuckle'))))
    for a, b in sorted(extra - disabled):
        ET.SubElement(srdf, 'disable_collisions', link1=a, link2=b, reason='Adjacent, fixed, or closed linkage')
    return ET.tostring(srdf, encoding='unicode')


def moveit_parameters(robot_description, controller_file):
    parameters = yaml.safe_load(Path(controller_file).read_text())
    parameters.update({
        'use_sim_time': False,
        'robot_description': robot_description,
        'robot_description_semantic': semantics(robot_description),
        'robot_description_kinematics': {'ur_manipulator': {
            'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
            'kinematics_solver_search_resolution': 0.005, 'kinematics_solver_timeout': 0.2}},
        'robot_description_planning': {'joint_limits': {
            name: {'has_velocity_limits': True, 'max_velocity': 0.5,
                   'has_acceleration_limits': True, 'max_acceleration': 0.5} for name in JOINTS}},
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.05,
            'planner_configs': {'RRTConnect': {'type': 'geometric::RRTConnect', 'range': 0.0}},
            'ur_manipulator': {'planner_configs': ['RRTConnect'], 'longest_valid_segment_fraction': 0.005}},
        'publish_robot_description_semantic': True,
    })
    return parameters


def primitive_parameters(cell):
    return {'use_sim_time': False,
            'manipulation.object_dimensions': [float(x) for x in cell['object_dimensions']],
            'manipulation.object_offset_xyz': [float(x) for x in cell['object_offset_xyz']],
            'manipulation.object_offset_rpy': [float(x) for x in cell['object_offset_rpy']]}
