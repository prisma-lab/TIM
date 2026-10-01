"""Plan and submit the separate parameterized two-object simulation experiment."""
import argparse
from pathlib import Path
import sys

import yaml

from task_planner.fast_downward import PlanningError, canonical_action, run_fast_downward
from task_planner.pddl_to_seed import publish_sequence

OBJECTS = ('red_connector', 'blue_peg')
LOCATIONS = ('red_pick', 'red_place', 'blue_pick', 'blue_place', 'home')
SUPPORTED_COMMANDS = frozenset(
    [f'move_a_b({location})' for location in LOCATIONS] +
    [f'{operation}({obj},{location})'
     for operation in ('pick', 'place') for obj in OBJECTS for location in LOCATIONS])


def load_mapping(path):
    data = yaml.safe_load(Path(path).expanduser().read_text())
    if not isinstance(data, dict) or set(data) != {'actions'} or not isinstance(data['actions'], dict):
        raise PlanningError('Mapping YAML must contain an actions dictionary')
    mapping = {}
    for action, command in data['actions'].items():
        if not isinstance(action, str) or not isinstance(command, str):
            raise PlanningError('Mapping keys and values must be strings')
        action = canonical_action(action)
        if action in mapping:
            raise PlanningError(f'Duplicate normalized mapping: {action}')
        if command not in SUPPORTED_COMMANDS:
            raise PlanningError(f'Unsupported two-object command: {command!r}')
        mapping[action] = command
    return mapping


def sequence_for(actions, mapping):
    commands = []
    for action in actions:
        key = canonical_action(action)
        if key not in mapping:
            raise PlanningError(f'No mapping for {key}')
        if mapping[key] not in SUPPORTED_COMMANDS:
            raise PlanningError(f'Unsupported two-object command: {mapping[key]!r}')
        commands.append(mapping[key])
    return 'hardSequence([' + ','.join(commands) + '])' if commands else None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', required=True, type=Path)
    parser.add_argument('--problem', required=True, type=Path)
    parser.add_argument('--mapping', required=True, type=Path)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--seed-topic', default='/seed_ur10_two_objects/stream')
    parser.add_argument('--wait-for-seed', type=float, default=10.0)
    parser.add_argument('--planner', default='fast-downward.py')
    parser.add_argument('--build')
    parser.add_argument('--search', default='astar(blind())')
    parser.add_argument('--timeout', type=float, default=30.0)
    args = parser.parse_args(argv)
    try:
        mapping = load_mapping(args.mapping)
        actions = run_fast_downward(args.domain.expanduser().read_text(), args.problem.expanduser().read_text(),
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
            print(f'Submitted once to {args.seed_topic}. Watch SEED for execution completion.')
        else:
            print('Plan only. Add --execute to submit it to SEED.')
        return 0
    except (OSError, PlanningError, yaml.YAMLError) as error:
        print(f'Two-object planning failed: {error}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    sys.exit(main())
