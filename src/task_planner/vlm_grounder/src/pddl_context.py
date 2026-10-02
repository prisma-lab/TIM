"""Read typed PDDL vocabulary and positive-fact problem contexts."""

import re
from dataclasses import dataclass


SYMBOL_PATTERN = r"[A-Za-z][A-Za-z0-9_-]*"


def _symbol(value, label, *, variable=False):
    pattern = r"\?" + SYMBOL_PATTERN if variable else SYMBOL_PATTERN
    if not isinstance(value, str) or not re.fullmatch(pattern, value):
        raise ValueError(f"Invalid PDDL {label}: {value!r}")


def parse_definition(text, kind):
    """Parse balanced lists, ignoring comments; require one named definition."""
    tokens = re.findall(r"\(|\)|[^\s()]+", re.sub(r";[^\n]*", "", text))
    roots, stack = [], []
    for token in tokens:
        if token == "(":
            node = []
            (stack[-1] if stack else roots).append(node)
            stack.append(node)
        elif token == ")":
            if not stack:
                raise ValueError("Unbalanced PDDL parentheses.")
            stack.pop()
        else:
            if not stack:
                raise ValueError("PDDL symbols must be inside a definition.")
            stack[-1].append(token)
    if stack:
        raise ValueError("Unbalanced PDDL parentheses.")
    if len(roots) != 1:
        raise ValueError("Expected exactly one PDDL definition.")
    definition = roots[0]
    if (len(definition) < 2 or not isinstance(definition[0], str)
            or definition[0].lower() != "define"
            or not isinstance(definition[1], list) or len(definition[1]) != 2
            or not isinstance(definition[1][0], str)
            or definition[1][0].lower() != kind):
        raise ValueError(f"Expected a PDDL {kind} definition: (define ({kind} NAME) ...).")
    _symbol(definition[1][1], f"{kind} name")
    return definition


def _sections(definition):
    sections = {}
    for section in definition[2:]:
        if not isinstance(section, list) or not section or not isinstance(section[0], str):
            raise ValueError("Malformed PDDL section.")
        name = section[0].lower()
        # Actions do not affect the declarations used by the JSON schema.
        if name in (":action", ":durative-action"):
            continue
        if name in sections:
            raise ValueError(f"Duplicate PDDL section: {name}")
        sections[name] = section[1:]
    return sections


def typed_symbols(tokens, *, variables=False):
    """Expand grouped declarations; untyped symbols have PDDL's object type."""
    declarations, pending = [], []
    tokens = iter(tokens)
    for token in tokens:
        if token == "-":
            entity_type = next(tokens, None)
            if not pending:
                raise ValueError("PDDL type annotation has no preceding names.")
            _symbol(entity_type, "type")
            declarations.extend((name, entity_type) for name in pending)
            pending = []
        else:
            _symbol(token, "variable" if variables else "symbol", variable=variables)
            pending.append(token)
    declarations.extend((name, "object") for name in pending)
    return declarations


@dataclass
class DomainVocabulary:
    name: str
    # Original spellings are retained for schemas and PDDL output.
    parents: dict
    predicates: dict

    def is_subtype(self, actual, expected):
        parents = {name.lower(): parent.lower() if parent else None
                   for name, parent in self.parents.items()}
        actual, expected = actual.lower(), expected.lower()
        while actual is not None:
            if actual == expected:
                return True
            actual = parents.get(actual)
        return False


def parse_domain(domain):
    """Read types, inheritance, and predicate signatures from a supplied domain."""
    definition = parse_definition(domain, "domain")
    sections = _sections(definition)
    for unsupported in (":constants", ":functions"):
        if sections.get(unsupported):
            raise ValueError(f"Domain {unsupported} are not supported by the positive-fact JSON format.")
    parents = {}
    seen = set()
    for name, parent in typed_symbols(sections.get(":types", [])):
        if name.lower() in seen:
            raise ValueError(f"Duplicate PDDL type: {name}")
        seen.add(name.lower())
        parents[name] = None if name.lower() == "object" and parent.lower() == "object" else parent
    if "object" not in seen:
        parents["object"] = None
    lookup = {name.lower(): parent.lower() if parent else None for name, parent in parents.items()}
    if lookup["object"] is not None:
        raise ValueError("PDDL's root object type cannot have a parent.")
    for name in lookup:
        ancestors = set()
        current = name
        while current is not None:
            if current not in lookup:
                raise ValueError(f"Unknown parent type: {current}")
            if current in ancestors:
                raise ValueError(f"Cyclic PDDL type hierarchy at: {current}")
            ancestors.add(current)
            current = lookup[current]

    predicates = {}
    seen = set()
    for declaration in sections.get(":predicates", []):
        if not isinstance(declaration, list) or not declaration:
            raise ValueError("Malformed PDDL predicate declaration.")
        name = declaration[0]
        _symbol(name, "predicate")
        if name.lower() in seen:
            raise ValueError(f"Duplicate PDDL predicate: {name}")
        seen.add(name.lower())
        args = typed_symbols(declaration[1:], variables=True)
        if len({arg.lower() for arg, _ in args}) != len(args):
            raise ValueError(f"Duplicate predicate variable: {name}")
        for _, entity_type in args:
            if entity_type.lower() not in lookup:
                raise ValueError(f"Unknown predicate argument type: {entity_type}")
        predicates[name] = tuple(entity_type for _, entity_type in args)
    if not predicates:
        raise ValueError("Domain must declare at least one predicate.")
    return DomainVocabulary(definition[1][1], parents, predicates)


def context_from_pddl(problem, domain_name):
    """Use a problem's declarations, retained init, and positive conjunctive goal."""
    definition = parse_definition(problem, "problem")
    sections = _sections(definition)
    if set(sections) - {":domain", ":objects", ":init", ":goal"}:
        raise ValueError("Problem contexts support only :domain, :objects, :init, and :goal.")
    supplied_domain = sections.get(":domain")
    if (not supplied_domain or len(supplied_domain) != 1
            or not isinstance(supplied_domain[0], str)
            or supplied_domain[0].lower() != domain_name.lower()):
        raise ValueError(f"Context problem must use domain {domain_name!r}.")

    def fact(expression):
        if not isinstance(expression, list) or not expression:
            raise ValueError("Context requires positive atomic facts.")
        predicate, *args = expression
        _symbol(predicate, "predicate")
        if predicate.lower() in ("and", "or", "not", "forall", "exists", "imply", "when"):
            raise ValueError("Context requires positive atomic facts; logical expressions are unsupported.")
        for arg in args:
            _symbol(arg, "argument")
        return {"predicate": predicate, "args": args}

    goal = sections.get(":goal", [])
    if len(goal) != 1 or not isinstance(goal[0], list) or not goal[0]:
        raise ValueError("Context requires one positive goal or an (and ...) goal.")
    expression = goal[0]
    goals = expression[1:] if isinstance(expression[0], str) and expression[0].lower() == "and" else [expression]
    return {
        "objects": [{"name": name, "type": entity_type}
                    for name, entity_type in typed_symbols(sections.get(":objects", []))],
        "init": [fact(expression) for expression in sections.get(":init", [])],
        "goal": [fact(expression) for expression in goals],
    }
