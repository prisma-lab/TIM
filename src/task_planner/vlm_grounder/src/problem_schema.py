"""Domain-aware JSON schema, validation, and partial initial-state completion."""

import re

if __package__:
    from .pddl_context import DomainVocabulary, parse_domain
else:
    from pddl_context import DomainVocabulary, parse_domain


OBJECT_TYPES = ("robot", "object", "location")
PREDICATE_SIGNATURES = {
    "eeAt": ("robot", "location"),
    "objectAt": ("object", "location"),
    "gripperHolding": ("robot", "object"),
    "gripperEmpty": ("robot",),
    "clearLoc": ("location",),
}
IDENTIFIER_PATTERN = r"[A-Za-z][A-Za-z0-9_-]*"

def validate_identifier(name, label="identifier"):
    """Allow only single PDDL symbols, including names such as blue_object."""
    if not isinstance(name, str) or not re.fullmatch(IDENTIFIER_PATTERN, name):
        raise ValueError(f"Invalid PDDL {label}: {name!r}")

GENERATION_MODES = ("full", "init-only")


def get_vocabulary(domain=None):
    """Keep the legacy pick-and-place API default; callers can supply a domain."""
    if domain is None:
        return DomainVocabulary(
            "manipulator-pick-place",
            {name: None for name in OBJECT_TYPES},
            PREDICATE_SIGNATURES,
        )
    if isinstance(domain, DomainVocabulary):
        return domain
    return parse_domain(domain)

def validate_mode(mode):
    if mode not in GENERATION_MODES:
        raise ValueError(f"Unknown generation mode: {mode!r}; choose from {GENERATION_MODES}")

def make_schema(mode="full", domain=None):
    """Select the fields the model must generate; full remains the default."""
    validate_mode(mode)
    vocabulary = get_vocabulary(domain)
    arities = [len(args) for args in vocabulary.predicates.values()]
    fact = {
        "type": "object",
        "properties": {
            "predicate": {"type": "string", "enum": list(vocabulary.predicates)},
            "args": {
                "type": "array",
                "items": {"type": "string", "pattern": "^" + IDENTIFIER_PATTERN + "$"},
                "minItems": min(arities),
                "maxItems": max(arities),
            },
        },
        "required": ["predicate", "args"],
        "additionalProperties": False,
    }
    schema = {
        "type": "object",
        "properties": {
            "objects": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "name": {"type": "string", "pattern": "^" + IDENTIFIER_PATTERN + "$"},
                        "type": {"type": "string", "enum": list(vocabulary.parents)},
                    },
                    "required": ["name", "type"],
                    "additionalProperties": False,
                },
            },
            "init": {"type": "array", "items": fact},
            "goal": {"type": "array", "items": fact, "minItems": 1},
        },
        "required": ["objects", "init", "goal"],
        "additionalProperties": False,
    }

    if mode == "init-only":
        schema["properties"] = {"init": schema["properties"]["init"]}
        schema["required"] = ["init"]
    return schema

def validate_context(context, domain=None):
    """Validate fixed declarations, goal, and optional retained initial facts."""
    _require_keys(context, ("objects", "goal"), "Context", optional=("init",))
    validate_problem({**context, "init": context.get("init", [])}, domain)

def complete_init_problem(generated, context, domain=None):
    """Preserve retained facts, append new facts once, and reject contradictions."""
    validate_context(context, domain)
    _require_keys(generated, ("init",), "Generated initial state")
    validate_problem({**context, "init": generated["init"]}, domain)
    init, seen = [], set()
    for fact in context.get("init", []) + generated["init"]:
        key = (fact["predicate"].lower(), tuple(name.lower() for name in fact["args"]))
        if key not in seen:
            init.append(fact)
            seen.add(key)
    problem = {**context, "init": init}
    validate_problem(problem, domain)
    return problem

def _require_keys(value, keys, label, optional=()):
    if (not isinstance(value, dict) or not set(keys) <= set(value)
            or set(value) - set(keys) - set(optional)):
        suffix = f"; optional: {', '.join(optional)}" if optional else ""
        raise ValueError(f"{label} must be an object with these keys: {', '.join(keys)}{suffix}")

def validate_problem(problem, domain=None):
    """Check JSON structure, PDDL names, references, types, and conflicting facts."""
    _require_keys(problem, ("objects", "init", "goal"), "Problem")
    vocabulary = get_vocabulary(domain)
    allowed_types = {name.lower() for name in vocabulary.parents}
    signatures = {name.lower(): args for name, args in vocabulary.predicates.items()}
    for section in ("objects", "init", "goal"):
        if not isinstance(problem[section], list):
            raise ValueError(f"{section} must be an array.")
    if not problem["goal"]:
        raise ValueError("Goal must not be empty.")

    types = {}
    for entity in problem["objects"]:
        _require_keys(entity, ("name", "type"), "Entity")
        name, entity_type = entity["name"], entity["type"]
        validate_identifier(name, "entity name")
        if name.lower() in types:
            raise ValueError(f"Duplicate entity (PDDL ignores case): {name}")
        if not isinstance(entity_type, str) or entity_type.lower() not in allowed_types:
            raise ValueError(f"Unknown type: {entity_type!r}")
        types[name.lower()] = entity_type

    for section in ("init", "goal"):
        facts = set()
        for fact in problem[section]:
            _require_keys(fact, ("predicate", "args"), f"{section} fact")
            predicate, args = fact["predicate"], fact["args"]
            if not isinstance(predicate, str) or predicate.lower() not in signatures:
                raise ValueError(f"Unknown predicate: {predicate!r}")
            expected = signatures[predicate.lower()]
            if not isinstance(args, list) or len(args) != len(expected):
                raise ValueError(f"Wrong argument count or structure: {fact}")
            for name, expected_type in zip(args, expected):
                validate_identifier(name, "argument")
                actual_type = types.get(name.lower())
                if actual_type is None or not vocabulary.is_subtype(actual_type, expected_type):
                    raise ValueError(
                        f"{section}: {name!r} must be declared as {expected_type!r} in {fact}"
                    )
            facts.add((predicate.lower(), tuple(name.lower() for name in args)))

        if vocabulary.name.lower() == "manipulator-pick-place":
            occupied = {args[1] for predicate, args in facts if predicate == "objectat"}
            clear = {args[0] for predicate, args in facts if predicate == "clearloc"}
            if occupied & clear:
                raise ValueError(f"{section}: locations cannot be occupied and clear: {sorted(occupied & clear)}")
            if len(signatures.get("gripperempty", ())) == 0:
                # Serdar's single-gripper domain has no robot parameter.
                if (("gripperempty", ()) in facts
                        and any(predicate == "gripperholding" for predicate, _ in facts)):
                    raise ValueError(f"{section}: gripper cannot hold an object and be empty")
            else:
                # The legacy domain identifies each gripper by its robot.
                holding = {args[0] for predicate, args in facts if predicate == "gripperholding"}
                empty = {args[0] for predicate, args in facts if predicate == "gripperempty"}
                if holding & empty:
                    raise ValueError(f"{section}: grippers cannot hold an object and be empty: {sorted(holding & empty)}")
        elif vocabulary.name.lower() == "battery-pack-assembly":
            occupied = {args[1] for predicate, args in facts if predicate in ("at", "installedat")}
            free = {args[0] for predicate, args in facts if predicate == "free"}
            if occupied & free:
                raise ValueError(f"{section}: locations cannot be occupied and free: {sorted(occupied & free)}")
