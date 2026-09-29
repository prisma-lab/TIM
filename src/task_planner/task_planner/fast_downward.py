"""Fast Downward execution with isolated files and explicit failure handling."""

import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import tempfile


class PlanningError(RuntimeError):
    """The planner did not return a usable plan."""


_ACTION = re.compile(r'\([a-z][a-z0-9_-]*(?:\s+[a-z0-9_][a-z0-9_-]*)*\)', re.IGNORECASE)


def canonical_action(text):
    """Normalize one grounded PDDL action, without interpreting it as code."""
    text = text.strip().lower()
    if not _ACTION.fullmatch(text):
        raise PlanningError(f'Invalid grounded PDDL action: {text!r}')
    return '(' + ' '.join(text[1:-1].split()) + ')'


def parse_plan(text):
    """Ignore Fast Downward comments; an empty successful plan is valid."""
    actions = []
    for line in text.splitlines():
        action = line.split(';', 1)[0].strip()
        if action:
            actions.append(canonical_action(action))
    return actions


def default_build():
    """Support TIM's existing build layout and normal Fast Downward installs."""
    workspace = os.environ.get('ROS_WS')
    if workspace:
        build = Path(workspace) / 'build/builds/release/bin'
        if (build / 'downward').is_file():
            return str(build)
    return 'release'


def _stop(process):
    """Stop the driver and its translator/search subprocesses together."""
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.communicate(timeout=2)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.communicate()


def run_fast_downward(domain, problem, *, executable='fast-downward.py',
                      build=None, search='astar(lmcut())', timeout=30.0):
    """Return grounded actions or raise; never reuse a previous sas_plan file."""
    if not domain.strip() or not problem.strip():
        raise PlanningError('Both domain and problem must contain PDDL text')
    if not 0 < timeout < float('inf'):
        raise PlanningError('Planner timeout must be positive and finite')
    executable_path = shutil.which(executable)
    if executable_path is None:
        raise PlanningError(f'Fast Downward executable not found: {executable}')
    executable_path = str(Path(executable_path).resolve())
    build = build or default_build()
    if '/' in build:
        build = str(Path(build).expanduser().resolve())
    with tempfile.TemporaryDirectory(prefix='tim-pddl-') as directory:
        workspace = Path(directory)
        domain_path = workspace / 'domain.pddl'
        problem_path = workspace / 'problem.pddl'
        plan_path = workspace / 'sas_plan'
        domain_path.write_text(domain)
        problem_path.write_text(problem)
        command = [executable_path, '--build', build, '--plan-file', str(plan_path),
                   str(domain_path), str(problem_path), '--search', search]
        try:
            process = subprocess.Popen(command, cwd=workspace, stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, text=True, start_new_session=True)
        except OSError as error:
            raise PlanningError(f'Cannot start Fast Downward: {error}') from error
        try:
            stdout, stderr = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired as error:
            _stop(process)
            raise PlanningError(f'Fast Downward timed out after {timeout:g} seconds') from error
        except BaseException:
            _stop(process)
            raise
        if process.returncode != 0:
            details = (stderr.strip() or stdout.strip())[-4000:]
            raise PlanningError(f'Fast Downward failed (exit {process.returncode}):\n{details}')
        if not plan_path.is_file():
            raise PlanningError('Fast Downward exited successfully but produced no plan file')
        return parse_plan(plan_path.read_text())
