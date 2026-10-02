"""Run image -> validated problem.json -> problem.pddl in one command."""

import argparse
import json
from pathlib import Path

if __package__:
    from .pddl_context import context_from_pddl
else:
    from pddl_context import context_from_pddl
if __package__:
    from .ollama_client import DEFAULT_ENDPOINT, DEFAULT_MODEL, chat_with_image
else:
    from ollama_client import DEFAULT_ENDPOINT, DEFAULT_MODEL, chat_with_image
if __package__:
    from .problem_json_to_pddl import (
        DEFAULT_DOMAIN_PATH, DEFAULT_PROBLEM_NAME, PROJECT_ROOT, problem_to_pddl,
    )
else:
    from problem_json_to_pddl import (
        DEFAULT_DOMAIN_PATH, DEFAULT_PROBLEM_NAME, PROJECT_ROOT, problem_to_pddl,
    )
if __package__:
    from .problem_prompt import (
        DEFAULT_GOAL_DESC_PATH, DEFAULT_SCENE_DESC_PATH, DEFAULT_PREDICATE_DESC_PATH,
        build_prompt,
    )
else:
    from problem_prompt import (
        DEFAULT_GOAL_DESC_PATH, DEFAULT_SCENE_DESC_PATH, DEFAULT_PREDICATE_DESC_PATH,
        build_prompt,
    )
if __package__:
    from .problem_schema import (
        GENERATION_MODES, complete_init_problem, get_vocabulary, make_schema, validate_context,
        validate_identifier, validate_mode, validate_problem,
    )
else:
    from problem_schema import (
        GENERATION_MODES, complete_init_problem, get_vocabulary, make_schema, validate_context,
        validate_identifier, validate_mode, validate_problem,
    )

DEFAULT_JSON_OUTPUT = PROJECT_ROOT / "vlm_grounder" / "output" / "generated_problem.json"
DEFAULT_PDDL_OUTPUT = PROJECT_ROOT / "pddl" / "generated_problem.pddl"

def generate_problem(image_path, domain, *, scene_path=DEFAULT_SCENE_DESC_PATH,
                     goal_path=DEFAULT_GOAL_DESC_PATH,
                     model=DEFAULT_MODEL, endpoint=DEFAULT_ENDPOINT, timeout=600,
                     mode="full", context=None,
                     predicate_definitions_path=DEFAULT_PREDICATE_DESC_PATH):
    """Generate and validate JSON; return the problem and Ollama response metadata."""
    vocabulary = get_vocabulary(domain)
    result = chat_with_image(
        build_prompt(domain, scene_path=scene_path, goal_path=goal_path,
                     mode=mode, context=context,
                     predicate_definitions_path=predicate_definitions_path),
        Path(image_path), make_schema(mode, vocabulary),
        model=model, endpoint=endpoint, timeout=timeout,
    )
    content = result["message"]["content"]
    try:
        problem = json.loads(content)
        if mode == "init-only":
            problem = complete_init_problem(problem, context, vocabulary)
        else:
            validate_problem(problem, vocabulary)
    except ValueError as exc:
        raise ValueError(f"Invalid generated problem: {exc}\nModel output:\n{content}") from exc
    return problem, result


def run_pipeline(image_path, *, domain_path=DEFAULT_DOMAIN_PATH,
                 json_output=DEFAULT_JSON_OUTPUT, pddl_output=DEFAULT_PDDL_OUTPUT,
                 problem_name=DEFAULT_PROBLEM_NAME, scene_path=DEFAULT_SCENE_DESC_PATH,
                 goal_path=DEFAULT_GOAL_DESC_PATH, model=DEFAULT_MODEL,
                 endpoint=DEFAULT_ENDPOINT, timeout=600, mode="full", context=None,
                 context_path=None,
                 predicate_definitions_path=DEFAULT_PREDICATE_DESC_PATH):
    """Generate both files, validating the entire result before writing either."""
    validate_mode(mode)
    domain_path = Path(domain_path)
    domain = domain_path.read_text(encoding="utf-8")
    vocabulary = get_vocabulary(domain)
    domain_name = vocabulary.name
    if context_path is not None:
        if context is not None:
            raise ValueError("Supply either context or context_path, not both.")
        context_path = Path(context_path)
        text = context_path.read_text(encoding="utf-8")
        context = (context_from_pddl(text, domain_name) if context_path.suffix.lower() == ".pddl"
                   else json.loads(text))
    if mode == "init-only":
        validate_context(context, vocabulary)
    elif context is not None:
        raise ValueError("Context is only supported in init-only mode.")
    image_path, domain_path, json_output, pddl_output = Path(image_path), Path(domain_path), Path(json_output), Path(pddl_output)
    outputs = {json_output.resolve(), pddl_output.resolve()}
    predicate_definitions_path = Path(predicate_definitions_path)
    scene_path, goal_path = Path(scene_path), Path(goal_path)
    inputs = {image_path.resolve(), domain_path.resolve(), predicate_definitions_path.resolve(),
              scene_path.resolve(), goal_path.resolve()}
    if context_path is not None:
        inputs.add(context_path.resolve())
    if len(outputs) != 2 or outputs & inputs:
        raise ValueError("JSON and PDDL outputs must be distinct and must not overwrite input files.")
    if timeout <= 0:
        raise ValueError("Timeout must be positive.")
    validate_identifier(problem_name, "problem name")
    problem, result = generate_problem(
        image_path, domain, scene_path=scene_path, goal_path=goal_path, model=model,
        endpoint=endpoint, timeout=timeout, mode=mode, context=context,
        predicate_definitions_path=predicate_definitions_path,
    )
    pddl = problem_to_pddl(problem, domain_name, problem_name, domain=vocabulary)
    json_output.parent.mkdir(parents=True, exist_ok=True)
    pddl_output.parent.mkdir(parents=True, exist_ok=True)
    json_output.write_text(json.dumps(problem, indent=2) + "\n", encoding="utf-8")
    pddl_output.write_text(pddl, encoding="utf-8")
    return json_output, pddl_output, result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, help="Initial-state image path")
    parser.add_argument("--domain", type=Path, default=DEFAULT_DOMAIN_PATH)
    parser.add_argument("--output", type=Path, default=DEFAULT_JSON_OUTPUT,
                        help="JSON output path (default: project output/problem.json)")
    parser.add_argument("--pddl-output", type=Path, default=DEFAULT_PDDL_OUTPUT,
                        help="Default: JSON output path with .pddl suffix")
    parser.add_argument("--problem-name", default=DEFAULT_PROBLEM_NAME)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE_DESC_PATH,
                        help="Scene description file (default: descriptions/scene.txt)")
    parser.add_argument("--goal", type=Path, default=DEFAULT_GOAL_DESC_PATH,
                        help="Goal description file for full mode (default: descriptions/goal.txt)")
    parser.add_argument("--mode", choices=GENERATION_MODES, default="full",
                        help="Generate all sections (default) or only init using supplied context")
    parser.add_argument("--context", type=Path,
                        help='Init-only JSON with "objects", "goal", optional retained "init", '
                             'or a PDDL problem containing the retained initial facts')
    parser.add_argument("--predicate-definitions", type=Path,
                        default=DEFAULT_PREDICATE_DESC_PATH,
                        help="Predicate description text file (default: descriptions/predicates.txt)")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--endpoint", default=DEFAULT_ENDPOINT, help="Ollama chat URL")
    parser.add_argument("--timeout", type=float, default=600, help="HTTP timeout in seconds")
    args = parser.parse_args(argv)
    if args.mode == "init-only" and args.context is None:
        parser.error("--context is required with --mode init-only")
    if args.mode == "full" and args.context is not None:
        parser.error("--context requires --mode init-only")
    print("Generating problem JSON and PDDL...")
    try:
        json_output, pddl_output, result = run_pipeline(
            args.image, domain_path=args.domain, json_output=args.output,
            pddl_output=args.pddl_output, problem_name=args.problem_name,
            scene_path=args.scene, goal_path=args.goal, model=args.model,
            endpoint=args.endpoint, timeout=args.timeout, mode=args.mode,
            context_path=args.context, predicate_definitions_path=args.predicate_definitions,
        )
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f"Pipeline failed: {exc}\n")
    
    print(f"Saved JSON to {json_output.resolve()}")
    print(f"Saved PDDL to {pddl_output.resolve()}")
    print("Prompt tokens:", result.get("prompt_eval_count"))
    print("Generated tokens:", result.get("eval_count"))


if __name__ == "__main__":
    main()
