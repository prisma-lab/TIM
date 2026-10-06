"""Plan hardware moves and gripper operations, then optionally submit them to SEED."""

import argparse
from pathlib import Path
import re
import sys

from task_planner.fast_downward import PlanningError, canonical_action, run_fast_downward
from task_planner.pddl_to_seed import publish_sequence


LOCATION_NAME = re.compile(r'[a-z][a-z0-9_]*')


def command_for(action):
    """Translate the hardware action vocabulary without discarding arguments."""
    tokens = canonical_action(action)[1:-1].split()
    name, arguments = tokens[0], tokens[1:]

    if name == 'move' and len(arguments) == 1:
        location = arguments[0]
        if not LOCATION_NAME.fullmatch(location):
            raise PlanningError('Location names must start with a letter and use letters, digits or underscores')
        return f'move({location})'

    if name in ('pick', 'place') and not arguments:
        return name

    raise PlanningError(f'Unsupported hardware action {action!r}; use (move LOCATION), (pick), or (place)')


def sequence_for(actions):
    """Validate every action before constructing a sequential SEED task."""
    commands = [command_for(action) for action in actions]
    return 'hardSequence([' + ','.join(commands) + '])' if commands else None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', required=True, type=Path)
    parser.add_argument('--problem', required=True, type=Path)
    parser.add_argument('--execute', action='store_true', help='Submit the complete plan to SEED')
    parser.add_argument('--seed-topic', default='/seed_ur10_services/stream')
    parser.add_argument('--wait-for-seed', type=float, default=10.0)
    parser.add_argument('--planner', default='fast-downward.py')
    parser.add_argument('--build', help='Fast Downward build directory or build name')
    parser.add_argument('--search', default='astar(blind())')
    parser.add_argument('--timeout', type=float, default=30.0)
    args = parser.parse_args(argv)

    try:
        actions = run_fast_downward(
            args.domain.expanduser().read_text(), args.problem.expanduser().read_text(),
            executable=args.planner, build=args.build, search=args.search, timeout=args.timeout)
        sequence = sequence_for(actions)
        if sequence is None:
            print('The PDDL goal is already satisfied: empty plan, no commands sent.')
            return 0

        print('Fast Downward plan:')
        for action in actions:
            print('  ' + action)
        print('SEED sequence:\n  ' + sequence, flush=True)
        if args.execute:
            publish_sequence(sequence, args.seed_topic, args.wait_for_seed)
            print(f'Submitted to {args.seed_topic}. Watch SEED and the primitive status topic for completion.')
        else:
            print('Plan only. Add --execute to submit it to SEED.')
        return 0
    except (OSError, PlanningError) as error:
        print(f'Hardware planning failed: {error}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    sys.exit(main())
