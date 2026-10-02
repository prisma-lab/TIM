# Image-to-planning-problem pipeline

From this directory, with Ollama running and `qwen3-vl:8b-instruct` available:

```bash
python3 src/vlm_generate_problem.py resource/example_problem_image.png
```

This calls Ollama once, validates its JSON, then writes `output/problem.json` and
`output/problem.pddl` in the project directory. PDDL conversion is deterministic.
The command stops after writing the two files; it does not run a planner.
The default domain is `resource/domain.pddl`. Default domain and output paths
are resolved relative to the project directory, regardless of the working
directory. Explicit relative paths are resolved from the working directory.
Output directories are created automatically.

The original `image`, `--domain`, and `--output` arguments remain available.
`--output` selects the JSON path; the PDDL path defaults to the same path with
a `.pddl` suffix. Use `--pddl-output` to choose a different PDDL path.

```bash
python3 src/vlm_generate_problem.py resource/example_problem_image.png \
  --domain resource/domain.pddl --output output/problem.json --pddl-output output/problem.pddl \
  --goal descriptions/goal.txt
```

`--scene` and `--goal` select UTF-8 description files. Their defaults are
`descriptions/scene.txt` and `descriptions/goal.txt`; edit these files to change
the scene or task. The default scene has two objects and five locations. `--model`, `--endpoint`,
`--timeout`, and `--problem-name` are also configurable; see `--help`.
The response schema and validation use the types, type hierarchy, and predicate
signatures declared in `--domain`. The default remains the manipulator
pick-and-place domain; `resource/crf_domain.pddl` provides the battery assembly
domain. Select matching scene and predicate descriptions when changing domains.

Convert an existing JSON file without Ollama:

```bash
python3 src/problem_json_to_pddl.py output/problem.json
```

Modules:

- `src/vlm_generate_problem.py`: single-run CLI and orchestration.
- `src/problem_prompt.py`: prompt construction and description-file loading.
- `src/ollama_client.py`: image encoding, HTTP request, and response handling.
- `src/problem_schema.py`: domain-aware JSON schema, validation, and init merging.
- `src/pddl_context.py`: domain declarations and retained facts from PDDL problems.
- `src/problem_json_to_pddl.py`: PDDL serialization and standalone conversion CLI.

Validation checks JSON structure, PDDL identifiers, declarations, predicate
argument types (including inherited types), and occupied/clear or holding/empty
conflicts in the manipulator domain. Battery assembly validation rejects `free`
locations that also have `at` or `installedAt` facts. It does not prove a problem
is solvable or validate action definitions.
Both representations are validated before either output is written.

Run the offline regression checks:

```bash
python3 -m unittest discover -s src -p 'test_*.py' -v
```

The model response is mocked in these tests; no Ollama service is required.

To ask the VLM for only missing initial facts, supply exact typed objects, goal
facts, and optionally known initial facts in a context JSON file, then select
`--mode init-only`:

```json
{
  "objects": [
    {"name": "robot", "type": "robot"},
    {"name": "blue_object", "type": "object"},
    {"name": "home", "type": "location"},
    {"name": "blue_initial", "type": "location"},
    {"name": "right_corner", "type": "location"}
  ],
  "init": [
    {"predicate": "gripperEmpty", "args": ["robot"]}
  ],
  "goal": [
    {"predicate": "objectAt", "args": ["blue_object", "right_corner"]}
  ]
}
```

Save this as `context.json`, adjusting declarations to include every entity
needed in your scene. Keep known facts in `init`; remove facts you want grounded
from the image. Omitting `init` is equivalent to an empty list. Then run:

```bash
python3 src/vlm_generate_problem.py resource/example_problem_image.png \
  --mode init-only --context context.json --output output/problem.json
```

The VLM receives the declarations, goals, and retained initial facts and returns
only `{"init": [...]}` with additional facts. Supplied objects, goals, and init
are validated before the request. Objects and goals remain fixed, and retained
init facts are merged with generated facts without duplicates (PDDL ignores
case). Conflicts in the merged state fail before output files are written.
An empty generated init preserves all retained facts. Init facts are checked
against the supplied declarations. `--goal` applies to full mode; init-only mode
uses the goal facts in the context file. `--scene` selects a description file with extra grounding
hints in either mode. Init-only mode does not load the goal description file;
the supplied context defines the goal and all allowed entity names.

You can also pass a partially edited PDDL problem as `--context`. Its objects
and goal are fixed; every fact left in `:init` is retained automatically. The
problem's domain must match `--domain`. JSON context has exactly `objects` and
`goal`, plus optional `init`; additional keys are rejected.

For the CRF example, remove only the unknown facts from
`resource/crf_example_problem.pddl`, then run from this directory:

```bash
python3 src/vlm_generate_problem.py resource/crf_problem.jpg \
  --domain resource/crf_domain.pddl \
  --mode init-only --context resource/crf_example_problem.pddl \
  --scene descriptions/crf/scene.txt \
  --predicate-definitions descriptions/crf/predicates.txt \
  --problem-name assemble-battery-pack --output output/crf_problem.json
```

This writes `output/crf_problem.json` and `output/crf_problem.pddl` while
preserving the source problem. `resource/context.json` is a complete JSON example
with the same 14 typed objects, 14 initial facts, and two goals as the original
CRF problem. To use JSON instead, remove unknown facts from its `init` list and
replace the command's context argument with `--context resource/context.json`.
Editing the PDDL does not change the JSON example automatically.

The supported problem format contains typed objects, positive atomic init
facts, and a positive atomic or conjunctive goal. Negative facts, numeric
fluents, quantified/disjunctive goals, domain constants, and `either` types are
not supported. Domain actions may use ADL, as in the CRF domain; only their
declared types and predicates are used for grounding validation.

Use `--mode full` (or omit `--mode`) for the original workflow that generates
objects, init, and goal together. `--context` is accepted only in init-only mode.

For Python callers, `generate_problem(..., mode="init-only", context=context)`
and `run_pipeline(..., mode="init-only", context=context)` accept the same
context dictionary. `run_pipeline` also accepts `context_path`. Returned and
saved problems always contain all three sections. `make_schema("init-only", domain)`
exposes the smaller response schema; `make_schema(domain=domain)` keeps the full
schema. These accept PDDL domain text. Without a domain, the schema and validation
helpers retain the original manipulator defaults. `problem_to_pddl` accepts a
`domain=` keyword for validating and rendering CRF or other domain vocabularies.

Scene, goal, and predicate descriptions live in `descriptions/`. Defaults are
`scene.txt`, `goal.txt`, and `predicates.txt`. Edit them or choose other files:

```bash
python3 src/vlm_generate_problem.py resource/example_problem_image.png \
  --scene descriptions/scene.txt --goal descriptions/goal.txt \
  --predicate-definitions descriptions/predicates.txt

python3 src/vlm_generate_problem.py resource/example_problem_image.png \
  --mode init-only --context context.json \
  --predicate-definitions descriptions/predicates.txt
```

Default description paths are relative to the project, independent of the
working directory. Explicit paths are relative to the working directory.
Missing, empty, or unreadable descriptions fail before calling the VLM.
Output paths must not overwrite any description or other input file.

Descriptions customize the natural-language prompt. Allowed predicate names
and argument types come from the supplied domain; domain-specific consistency
rules live in `src/problem_schema.py`. Changing descriptions does not add new
predicates.

Python callers can pass `scene_path`, `goal_path`, and
`predicate_definitions_path` to `run_pipeline`, `generate_problem`, and
`build_prompt`. These replace the previous inline scene/goal/predicate text
arguments. The CLI `--scene` and `--goal` now accept file paths rather than text.
