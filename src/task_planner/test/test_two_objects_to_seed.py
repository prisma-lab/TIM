"""PDDL ordering and command identity for the separate simulation experiment."""
from pathlib import Path
import shutil

import pytest

from task_planner import two_objects_to_seed as bridge
from task_planner.fast_downward import PlanningError, run_fast_downward
from task_planner.pddl_to_seed import load_mapping as original_mapping

PACKAGE = Path(__file__).resolve().parents[1]
DOMAIN = PACKAGE / 'pddl/two_objects_domain.pddl'
PROBLEM = PACKAGE / 'pddl/two_objects_problem.pddl'
MAPPING = PACKAGE / 'config/two_objects_action_mapping.yaml'
ACTIONS = ['(move_a_b home red_pick)', '(pick red_connector red_pick)',
           '(move_a_b red_pick red_place)', '(place red_connector red_place)',
           '(move_a_b red_place blue_pick)', '(pick blue_peg blue_pick)',
           '(move_a_b blue_pick blue_place)', '(place blue_peg blue_place)']
SEQUENCE = ('hardSequence([move_a_b(red_pick),pick(red_connector,red_pick),'
            'move_a_b(red_place),place(red_connector,red_place),'
            'move_a_b(blue_pick),pick(blue_peg,blue_pick),'
            'move_a_b(blue_place),place(blue_peg,blue_place)])')


def test_preserves_object_and_location_parameters():
    assert bridge.sequence_for(ACTIONS, bridge.load_mapping(MAPPING)) == SEQUENCE


def test_original_command_allowlist_is_unchanged():
    with pytest.raises(PlanningError, match='Unsupported'):
        original_mapping(MAPPING)


@pytest.mark.parametrize('command', ['pick', 'place', 'pick(unknown,blue_pick)',
                                     'pick(red_connector,unknown)',
                                     'pick(red_connector,red_pick),halt'])
def test_rejects_unsupported_or_injected_commands(tmp_path, command):
    path = tmp_path / 'mapping.yaml'
    path.write_text(f'actions:\n  "(pick red_connector red_pick)": "{command}"\n')
    with pytest.raises(PlanningError, match='Unsupported'):
        bridge.load_mapping(path)


def test_validates_whole_plan_before_publication(monkeypatch):
    published = []
    monkeypatch.setattr(bridge, 'run_fast_downward', lambda *a, **kw: ACTIONS + ['(unknown)'])
    monkeypatch.setattr(bridge, 'publish_sequence', lambda *a: published.append(a))
    assert bridge.main(['--domain', str(DOMAIN), '--problem', str(PROBLEM),
                        '--mapping', str(MAPPING), '--execute']) == 1
    assert not published


@pytest.mark.parametrize('execute', [False, True])
def test_uses_separate_seed_topic_only_when_requested(monkeypatch, execute):
    published = []
    monkeypatch.setattr(bridge, 'run_fast_downward', lambda *a, **kw: ACTIONS)
    monkeypatch.setattr(bridge, 'publish_sequence', lambda *a: published.append(a))
    args = ['--domain', str(DOMAIN), '--problem', str(PROBLEM), '--mapping', str(MAPPING)]
    if execute:
        args.append('--execute')
    assert bridge.main(args) == 0
    assert published == ([(SEQUENCE, '/seed_ur10_two_objects/stream', 10.)] if execute else [])


@pytest.mark.skipif(not shutil.which('fast-downward.py'), reason='Fast Downward unavailable')
def test_actual_planner_orders_red_before_blue_and_obeys_changed_order():
    domain, problem = DOMAIN.read_text(), PROBLEM.read_text()
    # The problem also requires the arm to return home after both placements.
    assert run_fast_downward(domain, problem, search='astar(blind())') == (
        ACTIONS + ['(move_a_b blue_place home)'])
    reverse = problem.replace('(before red_connector blue_peg)', '(before blue_peg red_connector)')
    actions = run_fast_downward(domain, reverse, search='astar(blind())')
    assert actions.index('(place blue_peg blue_place)') < actions.index('(pick red_connector red_pick)')
    # A changed plan must get a corresponding mapping; no silent argument dropping.
    with pytest.raises(PlanningError, match='No mapping'):
        bridge.sequence_for(actions, bridge.load_mapping(MAPPING))


def test_empty_plan_does_not_submit(monkeypatch):
    monkeypatch.setattr(bridge, 'run_fast_downward', lambda *a, **kw: [])
    def unexpected(*args):
        pytest.fail('Empty plan must not be published')
    monkeypatch.setattr(bridge, 'publish_sequence', unexpected)
    assert bridge.main(['--domain', str(DOMAIN), '--problem', str(PROBLEM),
                        '--mapping', str(MAPPING), '--execute']) == 0
