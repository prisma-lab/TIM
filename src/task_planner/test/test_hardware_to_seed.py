"""Hardware plans use the service vocabulary without per-location mappings."""
from pathlib import Path
import shutil

import pytest

from task_planner import hardware_to_seed
from task_planner.fast_downward import PlanningError, run_fast_downward


PDDL = Path(__file__).resolve().parents[1] / 'pddl/hardware'
ACTIONS = ['(move pick_approach)', '(move pick)', '(pick)',
           '(move pick_approach)', '(move place_approach)', '(move place)',
           '(place)', '(move place_approach)']
SEQUENCE = ('hardSequence([move(pick_approach),move(pick),pick,move(pick_approach),'
            'move(place_approach),move(place),place,move(place_approach)])')


def test_requested_hardware_sequence():
    assert hardware_to_seed.sequence_for(ACTIONS) == SEQUENCE


def test_location_names_are_not_hardcoded():
    assert hardware_to_seed.command_for('(MOVE inspection_station_27)') == 'move(inspection_station_27)'


@pytest.mark.parametrize('action,command', [('(pick )', 'pick'), ('(  PLACE   )', 'place')])
def test_parameterless_planner_actions_allow_padding(action, command):
    assert hardware_to_seed.command_for(action) == command


@pytest.mark.parametrize('action', [
    '(move)', '(move home pick)', '(pick workpiece)', '(place workpiece place)',
    '(move_a_b pick)', '(unknown pick)', '(move bad-name)', '(move 123)',
    '(move pick),halt', '(move pick);halt',
])
def test_rejects_unsupported_actions_without_dropping_arguments(action):
    with pytest.raises(PlanningError):
        hardware_to_seed.sequence_for([action])


def test_cli_checks_whole_plan_before_publishing(monkeypatch):
    published = []
    monkeypatch.setattr(hardware_to_seed, 'run_fast_downward', lambda *args, **kwargs: ACTIONS + ['(unknown)'])
    monkeypatch.setattr(hardware_to_seed, 'publish_sequence', lambda *args: published.append(args))
    assert hardware_to_seed.main([
        '--domain', str(PDDL / 'domain.pddl'), '--problem', str(PDDL / 'problem.pddl'), '--execute']) == 1
    assert published == []


@pytest.mark.parametrize('execute', [False, True])
def test_cli_only_submits_when_requested(monkeypatch, execute):
    published = []
    monkeypatch.setattr(hardware_to_seed, 'run_fast_downward', lambda *args, **kwargs: ACTIONS)
    monkeypatch.setattr(hardware_to_seed, 'publish_sequence', lambda *args: published.append(args))
    args = ['--domain', str(PDDL / 'domain.pddl'), '--problem', str(PDDL / 'problem.pddl')]
    if execute:
        args.append('--execute')
    assert hardware_to_seed.main(args) == 0
    assert published == ([(SEQUENCE, '/seed_ur10_services/stream', 10.0)] if execute else [])


def test_empty_plan_sends_nothing(monkeypatch):
    published = []
    monkeypatch.setattr(hardware_to_seed, 'run_fast_downward', lambda *args, **kwargs: [])
    monkeypatch.setattr(hardware_to_seed, 'publish_sequence', lambda *args: published.append(args))
    assert hardware_to_seed.main([
        '--domain', str(PDDL / 'domain.pddl'), '--problem', str(PDDL / 'problem.pddl'), '--execute']) == 0
    assert published == []


@pytest.mark.skipif(not shutil.which('fast-downward.py'), reason='Fast Downward unavailable')
def test_fast_downward_example_and_changed_goal():
    domain = (PDDL / 'domain.pddl').read_text()
    problem = (PDDL / 'problem.pddl').read_text()
    assert run_fast_downward(domain, problem, search='astar(blind())') == ACTIONS
    changed = problem.replace('(and (placed) (hand-empty) (arm-at place_approach))', '(holding)')
    assert run_fast_downward(domain, changed, search='astar(blind())') == ACTIONS[:3]
