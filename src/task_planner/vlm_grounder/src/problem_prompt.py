"""Grounding prompt"""

import json
from pathlib import Path

if __package__:
    from .problem_schema import get_vocabulary, make_schema, validate_context, validate_mode
else:
    from problem_schema import get_vocabulary, make_schema, validate_context, validate_mode

if __package__:
    from .problem_json_to_pddl import PROJECT_ROOT
else:
    from problem_json_to_pddl import PROJECT_ROOT

DESC_DIR = PROJECT_ROOT / "resource" / "descriptions"
DEFAULT_PREDICATE_DESC_PATH = DESC_DIR / "predicates.txt"
DEFAULT_SCENE_DESC_PATH = DESC_DIR / "scene.txt"
DEFAULT_GOAL_DESC_PATH = DESC_DIR / "goal.txt"

def load_definitions(path):
    """Load nonempty UTF-8 text; reject missing or empty description files."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
            if not text.strip():
                raise ValueError(f"File is empty: {path}")
            return text
    except FileNotFoundError:
        raise ValueError(f"File not found: {path}")


def build_prompt(domain, *, mode="full", context=None,
                 scene_path=DEFAULT_SCENE_DESC_PATH,
                 goal_path=DEFAULT_GOAL_DESC_PATH,
                 predicate_definitions_path=DEFAULT_PREDICATE_DESC_PATH):
    validate_mode(mode)
    vocabulary = get_vocabulary(domain)

    scene = load_definitions(scene_path)
    goal = load_definitions(goal_path) if mode == "full" else None
    predicate_definitions = load_definitions(predicate_definitions_path)

    scene_text = f"Scene description:\n{scene}"
    goal_text = f"Goal description:\n{goal}" if goal is not None else ""
    predicate_definitions_text = f"Predicate definitions:\n{predicate_definitions}" if predicate_definitions is not None else ""

    if mode == "init-only":
        validate_context(context, vocabulary)
        goal_text = f"Supplied objects, goal, and any retained init facts (authoritative):\n{json.dumps(context)}"
        fields = """Return JSON with only:
        - init: additional positive facts true in the initial state.
        Do not generate or change objects or goal. Use only the supplied objects.
        Supplied init facts are known true and will be preserved automatically.
        Fill in the missing initial facts; do not contradict or repeat retained facts.
        A missing init fact is unknown, not evidence that it is false or true.
        Use the image and explicit descriptions to determine additional facts."""
    else:
        if context is not None:
            raise ValueError("Context is only supported in init-only mode.")
        fields = """Return JSON with:
        - objects: all named entities, each with a name and a type from the domain.
        - init: positive facts true in the initial state.
        - goal: positive facts that must all hold in the goal state."""

    return f"""

Follow the instructions below to ground the attached initial-state image into a symbolic planning problem.

<domain>
{domain}
</domain>

{predicate_definitions_text}

{scene_text}

{goal_text}

{fields}

Each fact has a predicate and an ordered args array.
Use consistent entity names throughout and valid PDDL names beginning with a letter.
Every argument must reference an entity declared in objects.
Return only JSON, without explanations or Markdown.

Required JSON schema:
{json.dumps(make_schema(mode, vocabulary))}
""".strip()
