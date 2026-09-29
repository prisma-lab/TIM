"""Plan a single-connector UR10 task and submit its sequence to SEED."""

import argparse
from pathlib import Path
import sys
import time

import yaml

from task_planner.fast_downward import PlanningError, canonical_action, run_fast_downward


# These are the commands implemented by the current three-plugin UR10 demo.
SUPPORTED_COMMANDS = frozenset(('move_a_b(pick)', 'pick', 'move_a_b(place)', 'place'))
DEFAULT_MAPPING = {
    '(move_a_b pick)': 'move_a_b(pick)',
    '(pick)': 'pick',
    '(move_a_b place)': 'move_a_b(place)',
    '(place)': 'place',
}


def load_mapping(path):
    """Require exact grounded-action mappings: never silently drop arguments."""
    if path is None:
        return dict(DEFAULT_MAPPING)
    data = yaml.safe_load(Path(path).expanduser().read_text())
    if not isinstance(data, dict) or set(data) != {'actions'} or not isinstance(data['actions'], dict):
        raise PlanningError('Mapping YAML must contain an actions dictionary')
    mapping = {}
    for action, command in data['actions'].items():
        if not isinstance(action, str) or not isinstance(command, str):
            raise PlanningError('Mapping keys and values must be strings')
        key = canonical_action(action)
        if key in mapping:
            raise PlanningError(f'Duplicate normalized mapping: {key}')
        if command not in SUPPORTED_COMMANDS:
            raise PlanningError(f'Unsupported UR10 primitive in mapping: {command!r}')
        mapping[key] = command
    return mapping


def sequence_for(actions, mapping):
    """Validate the entire plan before constructing a SEED task expression."""
    commands = []
    for index, action in enumerate(actions, 1):
        key = canonical_action(action)
        if key not in mapping:
            raise PlanningError(f'Plan step {index} has no mapping: {key}')
        command = mapping[key]
        if command not in SUPPORTED_COMMANDS:
            raise PlanningError(f'Plan step {index} maps to an unsupported command: {command!r}')
        commands.append(command)
    return 'hardSequence([' + ','.join(commands) + '])' if commands else None


def publish_sequence(sequence, topic, wait_timeout):
    """Publish once after discovery. SEED owns subsequent execution/feedback."""
    import rclpy
    from rclpy.duration import Duration
    from std_msgs.msg import String

    if not 0 < wait_timeout < float('inf'):
        raise PlanningError('SEED discovery timeout must be positive and finite')
    rclpy.init(args=[])
    node = rclpy.create_node('pddl_to_seed')
    try:
        publisher = node.create_publisher(String, topic, 10)
        deadline = time.monotonic() + wait_timeout
        while publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise PlanningError(f'No SEED subscriber on {topic}; start SEED in this ROS domain')
            rclpy.spin_once(node, timeout_sec=0.1)
        publisher.publish(String(data=sequence))
        if not publisher.wait_for_all_acked(Duration(seconds=3)):
            raise PlanningError('SEED transport did not acknowledge the request; inspect SEED before retrying')
        # Give the receiving ROS callback time to run before the one-shot publisher exits.
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', required=True, type=Path, help='Domain PDDL file inside this container')
    parser.add_argument('--problem', required=True, type=Path, help='Problem PDDL file inside this container')
    parser.add_argument('--mapping', type=Path, help='YAML mapping exact grounded actions to UR10 commands')
    parser.add_argument('--execute', action='store_true', help='Send the valid plan to the running SEED instance')
    parser.add_argument('--seed-topic', default='/seed_ur10/stream')
    parser.add_argument('--wait-for-seed', type=float, default=10.0)
    parser.add_argument('--planner', default='fast-downward.py')
    parser.add_argument('--build', help='Fast Downward build directory or build name')
    parser.add_argument('--search', default='astar(lmcut())')
    parser.add_argument('--timeout', type=float, default=30.0)
    args = parser.parse_args(argv)
    try:
        mapping = load_mapping(args.mapping)
        actions = run_fast_downward(args.domain.expanduser().read_text(),
                                   args.problem.expanduser().read_text(),
                                   executable=args.planner, build=args.build,
                                   search=args.search, timeout=args.timeout)
        sequence = sequence_for(actions, mapping)
        if sequence is None:
            print('The PDDL goal is already satisfied: empty plan, no commands sent.')
            return 0
        print('Fast Downward plan:')
        for action in actions:
            print('  ' + action)
        print('SEED sequence:\n  ' + sequence, flush=True)
        if args.execute:
            publish_sequence(sequence, args.seed_topic, args.wait_for_seed)
            print(f'Submitted once to {args.seed_topic}. SEED now controls execution; '
                  'this message is not confirmation that the robot finished.')
        else:
            print('Plan only. Add --execute to submit it to SEED.')
        return 0
    except (OSError, PlanningError, yaml.YAMLError) as error:
        print(f'PDDL integration failed: {error}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    sys.exit(main())
