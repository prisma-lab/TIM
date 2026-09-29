"""Check the boundary between untrusted planner output and SEED commands."""

from pathlib import Path
import shutil

import pytest

from task_planner import pddl_to_seed
from task_planner.fast_downward import PlanningError, parse_plan, run_fast_downward

PACKAGE = Path(__file__).resolve().parents[1]
DOMAIN = PACKAGE / 'pddl/ur10_pick_place_domain.pddl'
PROBLEM = PACKAGE / 'pddl/ur10_pick_place_problem.pddl'
MAPPING = PACKAGE / 'config/ur10_action_mapping.yaml'
ACTIONS = ['(move-a-b home pick)', '(pick red-connector pick)',
           '(move-a-b pick place)', '(place red-connector place)']
SEQUENCE = 'hardSequence([move_a_b(pick),pick,move_a_b(place),place])'


def test_grounded_plan_maps_to_existing_seed_tasks():
    actions = parse_plan('\n'.join(ACTIONS) + '\n; cost = 4 (unit cost)\n')
    assert pddl_to_seed.sequence_for(actions, pddl_to_seed.load_mapping(MAPPING)) == SEQUENCE


def test_normalizes_case_and_whitespace():
    assert parse_plan('  (PICK   Red-Connector PICK) ; comment') == [ACTIONS[1]]


def test_empty_plan_has_no_sequence():
    assert pddl_to_seed.sequence_for(parse_plan('; cost = 0'), {}) is None


@pytest.mark.parametrize('text', [
    'Planner failed', '(pick);place', '(pick red_connector).',
    '(pick(red_connector))', '(pick),halt', '(pick) (place)',
])
def test_rejects_non_pddl_output(text):
    with pytest.raises(PlanningError):
        pddl_to_seed.sequence_for([text], pddl_to_seed.load_mapping(MAPPING))


@pytest.mark.parametrize('command', ['halt', 'pick,place', 'hardSequence([pick])'])
def test_rejects_unknown_or_injected_seed_commands(tmp_path, command):
    mapping = tmp_path / 'map.yaml'
    mapping.write_text(f'actions:\n  "(pick red-connector pick)": "{command}"\n')
    with pytest.raises(PlanningError, match='Unsupported'):
        pddl_to_seed.load_mapping(mapping)


def test_checks_object_arguments():
    with pytest.raises(PlanningError, match='no mapping'):
        pddl_to_seed.sequence_for(['(pick blue-peg pick)'], pddl_to_seed.load_mapping(MAPPING))


def test_cli_validates_entire_plan_before_publish(monkeypatch):
    published = []
    monkeypatch.setattr(pddl_to_seed, 'run_fast_downward',
                        lambda *args, **kwargs: ACTIONS + ['(unknown-action)'])
    monkeypatch.setattr(pddl_to_seed, 'publish_sequence', lambda *args: published.append(args))
    assert pddl_to_seed.main(['--domain', str(DOMAIN), '--problem', str(PROBLEM),
                             '--mapping', str(MAPPING), '--execute']) == 1
    assert not published


@pytest.mark.parametrize('execute', [False, True])
def test_cli_only_publishes_when_requested(monkeypatch, execute):
    published = []
    monkeypatch.setattr(pddl_to_seed, 'run_fast_downward', lambda *args, **kwargs: ACTIONS)
    monkeypatch.setattr(pddl_to_seed, 'publish_sequence', lambda *args: published.append(args))
    args = ['--domain', str(DOMAIN), '--problem', str(PROBLEM), '--mapping', str(MAPPING)]
    if execute:
        args.append('--execute')
    assert pddl_to_seed.main(args) == 0
    assert published == ([(SEQUENCE, '/seed_ur10/stream', 10.0)] if execute else [])


def test_cli_does_not_publish_already_satisfied_plan(monkeypatch):
    published = []
    monkeypatch.setattr(pddl_to_seed, 'run_fast_downward', lambda *args, **kwargs: [])
    monkeypatch.setattr(pddl_to_seed, 'publish_sequence', lambda *args: published.append(args))
    assert pddl_to_seed.main(['--domain', str(DOMAIN), '--problem', str(PROBLEM),
                             '--execute']) == 0
    assert not published


def fake_planner(tmp_path, body):
    script = tmp_path / 'fake-planner'
    script.write_text('#!/usr/bin/env python3\nimport sys\nfrom pathlib import Path\n' + body)
    script.chmod(0o755)
    return str(script)


def test_missing_plan_never_reuses_caller_plan(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    (tmp_path / 'sas_plan').write_text('(pick)\n')
    executable = fake_planner(tmp_path, 'sys.exit(0)\n')
    with pytest.raises(PlanningError, match='no plan file'):
        run_fast_downward('domain', 'problem', executable=executable)


def test_nonzero_exit_rejects_even_a_written_plan(tmp_path):
    executable = fake_planner(tmp_path,
        "Path(sys.argv[sys.argv.index('--plan-file') + 1]).write_text('(pick)')\n"
        'sys.exit(11)\n')
    with pytest.raises(PlanningError, match='exit 11'):
        run_fast_downward('domain', 'problem', executable=executable)


def test_timeout_stops_planner_child(tmp_path):
    marker = tmp_path / 'child-finished'
    body = (
        'import subprocess, time\n'
        f'code = "import time; from pathlib import Path; time.sleep(1.5); '
        f'Path({str(marker)!r}).touch()"\n'
        'subprocess.Popen([sys.executable, "-c", code])\n'
        'time.sleep(30)\n')
    executable = fake_planner(tmp_path, body)
    with pytest.raises(PlanningError, match='timed out'):
        run_fast_downward('domain', 'problem', executable=executable, timeout=0.5)
    # A child escaping cleanup would leave this marker after its delay.
    import time
    time.sleep(1.6)
    assert not marker.exists()


@pytest.mark.skipif(not shutil.which('fast-downward.py'), reason='Fast Downward unavailable')
def test_actual_fast_downward_success_empty_invalid_and_unsolvable():
    domain, problem = DOMAIN.read_text(), PROBLEM.read_text()
    assert run_fast_downward(domain, problem) == ACTIONS
    already_done = problem.replace('(object-at red-connector pick)',
                                   '(object-at red-connector place)')
    assert run_fast_downward(domain, already_done) == []
    # A later failure must not return the earlier successful plan.
    with pytest.raises(PlanningError):
        run_fast_downward(domain, problem.replace('(connected pick place)', ''))
    with pytest.raises(PlanningError):
        run_fast_downward('this is not PDDL', problem)
