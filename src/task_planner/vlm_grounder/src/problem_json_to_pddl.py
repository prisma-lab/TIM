"""Convert validated problem JSON to typed PDDL using the selected domain."""

import argparse
import json
import re
from pathlib import Path

if __package__:
    from .problem_schema import get_vocabulary, validate_identifier, validate_problem
else:
    from problem_schema import get_vocabulary, validate_identifier, validate_problem


PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent # task_planner package root
DEFAULT_DOMAIN_PATH = PROJECT_ROOT / "pddl" / "domain.pddl"
DEFAULT_PROBLEM_NAME = "generated-problem"
DEFAULT_PDDL_OUTPUT = PROJECT_ROOT / "pddl" / "generated_problem.pddl"

def get_domain_name(domain):
    """Read the domain name from its definition, ignoring PDDL comments."""
    without_comments = re.sub(r";[^\n]*", "", domain)
    match = re.match(
        r"\s*\(\s*define\s*\(\s*domain\s+([^\s()]+)\s*\)",
        without_comments, re.IGNORECASE,
    )
    if not match:
        raise ValueError("Expected a PDDL domain definition: (define (domain NAME) ...).")
    name = match.group(1)
    validate_identifier(name, "domain name")
    return name


def problem_to_pddl(problem, domain_name, problem_name=DEFAULT_PROBLEM_NAME, *, domain=None):
    """Render typed objects, positive initial facts, and a conjunctive goal."""
    vocabulary = get_vocabulary(domain)
    validate_problem(problem, vocabulary)
    validate_identifier(domain_name, "domain name")
    validate_identifier(problem_name, "problem name")
    if domain is not None and vocabulary.name.lower() != domain_name.lower():
        raise ValueError("Supplied domain name does not match the validation domain.")
    lines = [f"(define (problem {problem_name})", f"  (:domain {domain_name})", "  (:objects"]
    for entity_type in vocabulary.parents:
        names = [entity["name"] for entity in problem["objects"]
                 if entity["type"].lower() == entity_type.lower()]
        if names:
            lines.append(f"    {' '.join(names)} - {entity_type}")
    lines.extend(["  )", "  (:init"])
    for fact in problem["init"]:
        lines.append(f"    ({fact['predicate']} {' '.join(fact['args'])})")
    lines.extend(["  )", "  (:goal (and"])
    for fact in problem["goal"]:
        lines.append(f"    ({fact['predicate']} {' '.join(fact['args'])})")
    lines.extend(["  ))", ")"])
    return "\n".join(lines) + "\n"


def convert_problem_file(input_path, domain_path=DEFAULT_DOMAIN_PATH,
                         output_path=None, *, problem_name=DEFAULT_PROBLEM_NAME):
    input_path, domain_path = Path(input_path), Path(domain_path)
    output_path = Path(output_path) if output_path is not None else input_path.with_suffix(".pddl")
    if output_path.resolve() in {input_path.resolve(), domain_path.resolve()}:
        raise ValueError("PDDL output must not overwrite the input JSON or domain file.")
    problem = json.loads(input_path.read_text(encoding="utf-8"))
    domain = domain_path.read_text(encoding="utf-8")
    domain_name = get_domain_name(domain)
    pddl = problem_to_pddl(problem, domain_name, problem_name, domain=domain)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(pddl, encoding="utf-8")
    return output_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("problem", type=Path, help="Existing problem JSON file")
    parser.add_argument("--domain", type=Path, default=DEFAULT_DOMAIN_PATH)
    parser.add_argument("--output", type=Path, help="Default: input path with .pddl suffix")
    parser.add_argument("--problem-name", default=DEFAULT_PROBLEM_NAME)
    args = parser.parse_args(argv)
    try:
        output = convert_problem_file(args.problem, args.domain, args.output,
                                      problem_name=args.problem_name)
    except (OSError, ValueError) as exc:
        parser.exit(1, f"Conversion failed: {exc}\n")
    print(f"Saved to {output.resolve()}")


if __name__ == "__main__":
    main()
