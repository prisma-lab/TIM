"""Offline regression checks for conversion and the one-command pipeline."""

import copy
import io
import json
import tempfile
import unittest
import urllib.error
from pathlib import Path
from unittest.mock import patch

from ollama_client import chat_with_image
from problem_json_to_pddl import DEFAULT_DOMAIN_PATH, convert_problem_file, get_domain_name, problem_to_pddl
from problem_prompt import (
    DEFAULT_PREDICATE_DESC_PATH, DEFAULT_SCENE_DESC_PATH, DEFAULT_GOAL_DESC_PATH, build_prompt,
)
from problem_schema import make_schema, validate_problem
from vlm_generate_problem import DEFAULT_JSON_OUTPUT, main, run_pipeline


PROJECT_ROOT = Path(__file__).resolve().parent.parent
RESOURCE = PROJECT_ROOT / "resource"
OUTPUT = PROJECT_ROOT / "output"


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.problem = json.loads((OUTPUT / "problem.json").read_text(encoding="utf-8"))

    def response(self, problem=None):
        payload = {
            "message": {"content": json.dumps(self.problem if problem is None else problem)},
            "prompt_eval_count": 10,
            "eval_count": 20,
        }
        return io.BytesIO(json.dumps(payload).encode("utf-8"))

    def test_single_command_writes_both_files_and_sends_config(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "nested" / "task.json"
            pddl_output = Path(directory) / "pddl" / "task.pddl"
            scene_path = Path(directory) / "scene.txt"
            goal_path = Path(directory) / "goal.txt"
            scene_path.write_text("A custom scene", encoding="utf-8")
            goal_path.write_text("Move red left", encoding="utf-8")
            with patch("ollama_client.urllib.request.urlopen", return_value=self.response()) as request:
                main([
                    str(RESOURCE / "example_problem_image.png"), "--output", str(output),
                    "--pddl-output", str(pddl_output),
                    "--scene", str(scene_path), "--goal", str(goal_path),
                    "--model", "test-model", "--endpoint", "http://localhost:1234/api/chat",
                ])
            self.assertEqual(json.loads(output.read_text()), self.problem)
            pddl = pddl_output.read_text()
            self.assertIn("(:domain manipulator-pick-place)", pddl)
            self.assertIn("blue_object red_object - object", pddl)
            self.assertIn("(objectAt blue_object right_corner)", pddl)
            sent_request = request.call_args.args[0]
            payload = json.loads(sent_request.data)
            self.assertEqual(sent_request.full_url, "http://localhost:1234/api/chat")
            self.assertEqual(payload["model"], "test-model")
            self.assertIn("A custom scene", payload["messages"][0]["content"])
            self.assertIn("Goal description:\nMove red left", payload["messages"][0]["content"])
            self.assertTrue(payload["messages"][0]["images"][0])
            self.assertFalse(payload["stream"])
            self.assertIn("objects", payload["format"]["properties"])

    def test_init_only_preserves_context_and_requests_only_init(self):
        context = {key: copy.deepcopy(self.problem[key]) for key in ("objects", "goal")}
        context["goal"] = [{"predicate": "gripperEmpty", "args": ["robot"]}]
        generated = {"init": self.problem["init"]}
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "problem.json"
            context_path = Path(directory) / "context.json"
            context_path.write_text(json.dumps(context))
            with patch("ollama_client.urllib.request.urlopen", return_value=self.response(generated)) as request:
                main([
                    str(RESOURCE / "example_problem_image.png"), "--output", str(output),
                    "--mode", "init-only", "--context", str(context_path),
                ])
            complete = json.loads(output.read_text())
            self.assertEqual(complete, {**context, **generated})
            self.assertIn("(gripperEmpty robot)", output.with_suffix(".pddl").read_text())
            payload = json.loads(request.call_args.args[0].data)
            self.assertEqual(payload["format"], make_schema("init-only"))
            self.assertEqual(set(payload["format"]["properties"]), {"init"})
            self.assertEqual(payload["format"]["required"], ["init"])
            prompt = payload["messages"][0]["content"]
            self.assertIn(json.dumps(context), prompt)
            self.assertIn("Return JSON with only:", prompt)
            self.assertNotIn("Bring the blue object", prompt)

    def test_invalid_init_context_fails_before_model_call(self):
        invalid_contexts = [None, {}, {"objects": self.problem["objects"], "goal": []},
                            {"objects": [], "goal": self.problem["goal"]}]
        with patch("ollama_client.urllib.request.urlopen") as request:
            for context in invalid_contexts:
                with self.subTest(context=context), self.assertRaises(ValueError):
                    run_pipeline(RESOURCE / "example_problem_image.png",
                                 mode="init-only", context=context)
            with self.assertRaises(ValueError):
                run_pipeline(RESOURCE / "example_problem_image.png", mode="unknown")
            request.assert_not_called()

    def test_init_only_rejects_extra_sections_unknown_entities_and_conflicts(self):
        context = {key: self.problem[key] for key in ("objects", "goal")}
        invalid_responses = [self.problem,
                             {"init": [{"predicate": "gripperEmpty", "args": ["unknown"]}]},
                             {"init": self.problem["init"] + [
                                 {"predicate": "clearLoc", "args": ["blue_initial"]}]},
                             {"init": "invalid"}]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "problem.json"
            output.write_text("original")
            for generated in invalid_responses:
                with self.subTest(generated=generated):
                    with patch("ollama_client.urllib.request.urlopen", return_value=self.response(generated)):
                        with self.assertRaisesRegex(ValueError, "Invalid generated problem"):
                            run_pipeline(RESOURCE / "example_problem_image.png", mode="init-only",
                                         context=context, json_output=output)
                    self.assertEqual(output.read_text(), "original")
                    self.assertFalse(output.with_suffix(".pddl").exists())

    def test_context_file_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            context_path = Path(directory) / "context.json"
            context = {key: self.problem[key] for key in ("objects", "goal")}
            context_path.write_text(json.dumps(context))
            with patch("ollama_client.urllib.request.urlopen") as request:
                with self.assertRaises(ValueError):
                    run_pipeline(RESOURCE / "example_problem_image.png", mode="init-only",
                                 context_path=context_path, json_output=context_path)
                request.assert_not_called()

    def test_default_paths_are_anchored_to_project(self):
        self.assertEqual(DEFAULT_DOMAIN_PATH, RESOURCE / "domain.pddl")
        self.assertEqual(DEFAULT_JSON_OUTPUT, OUTPUT / "problem.json")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "nested" / "problem.pddl"
            result = convert_problem_file(OUTPUT / "problem.json", output_path=output)
            self.assertEqual(result, output)
            self.assertIn("(:domain manipulator-pick-place)", output.read_text())

    def test_invalid_model_output_preserves_existing_outputs(self):
        problem = copy.deepcopy(self.problem)
        problem["goal"][0]["args"][0] = "undeclared"
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "task.json"
            pddl_output = Path(directory) / "task.pddl"
            output.write_text("existing json")
            pddl_output.write_text("existing pddl")
            with patch("ollama_client.urllib.request.urlopen", return_value=self.response(problem)):
                with self.assertRaisesRegex(ValueError, "Invalid generated problem"):
                    run_pipeline(RESOURCE / "example_problem_image.png", json_output=output)
            self.assertEqual(output.read_text(), "existing json")
            self.assertEqual(pddl_output.read_text(), "existing pddl")

    def test_validation_rejects_malformed_and_unsafe_data(self):
        mutations = [
            lambda p: p.update(goal=[]),
            lambda p: p.update(objects={}),
            lambda p: p["objects"][0].update(name="robot) (:goal"),
            lambda p: p["objects"].append({"name": "ROBOT", "type": "robot"}),
            lambda p: p["goal"][0].update(predicate=[]),
            lambda p: p["goal"][0].update(args="blue_object"),
            lambda p: p["goal"][0].update(args=["robot", "right_corner"]),
            lambda p: p["init"].append({"predicate": "clearLoc", "args": ["blue_initial"]}),
            lambda p: p["init"].append({"predicate": "gripperHolding", "args": ["robot", "blue_object"]}),
        ]
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                problem = copy.deepcopy(self.problem)
                mutate(problem)
                with self.assertRaises(ValueError):
                    validate_problem(problem)

    def test_multiple_goals_and_empty_initial_state(self):
        self.problem["init"] = []
        self.problem["goal"].append({"predicate": "gripperEmpty", "args": ["robot"]})
        pddl = problem_to_pddl(self.problem, "manipulator-pick-place", "two-goals")
        self.assertIn("(:init\n  )", pddl)
        self.assertIn("(:goal (and\n    (objectAt blue_object right_corner)\n    (gripperEmpty robot)", pddl)

    def test_domain_name_comments_and_invalid_names(self):
        self.assertEqual(get_domain_name("; (domain wrong)\n(DEFINE (DOMAIN actual-domain))"), "actual-domain")
        with self.assertRaises(ValueError):
            get_domain_name("(define (problem wrong))")
        with self.assertRaises(ValueError):
            problem_to_pddl(self.problem, "valid", "invalid name")

    def test_output_collision_fails_before_model_call(self):
        with patch("ollama_client.urllib.request.urlopen") as request:
            with self.assertRaises(ValueError):
                run_pipeline(RESOURCE / "example_problem_image.png", json_output=RESOURCE / "domain.pddl")
            request.assert_not_called()

    def test_ollama_failures(self):
        responses = [{"error": "model missing"}, {"message": {}}, []]
        for response in responses:
            with self.subTest(response=response):
                with patch("ollama_client.urllib.request.urlopen", return_value=io.BytesIO(json.dumps(response).encode())):
                    with self.assertRaises(RuntimeError):
                        chat_with_image("prompt", RESOURCE / "example_problem_image.png", {})
        with patch("ollama_client.urllib.request.urlopen", side_effect=urllib.error.URLError("refused")):
            with self.assertRaisesRegex(RuntimeError, "Cannot reach Ollama"):
                chat_with_image("prompt", RESOURCE / "example_problem_image.png", {})

    def test_selected_predicate_file_in_both_modes(self):
        context = {key: self.problem[key] for key in ("objects", "goal")}
        definitions = "Custom grounding: objectAt(object, location) means observed placement."
        with tempfile.TemporaryDirectory() as directory:
            definitions_path = Path(directory) / "custom.txt"
            definitions_path.write_text(definitions, encoding="utf-8")
            context_path = Path(directory) / "context.json"
            context_path.write_text(json.dumps(context))
            for mode in ("full", "init-only"):
                with self.subTest(mode=mode):
                    generated = self.problem if mode == "full" else {"init": self.problem["init"]}
                    args = [str(RESOURCE / "example_problem_image.png"), "--mode", mode,
                            "--output", str(Path(directory) / f"{mode}.json"),
                            "--predicate-definitions", str(definitions_path)]
                    if mode == "init-only":
                        args.extend(["--context", str(context_path)])
                    with patch("ollama_client.urllib.request.urlopen", return_value=self.response(generated)) as request:
                        main(args)
                    payload = json.loads(request.call_args.args[0].data)
                    self.assertIn(definitions, payload["messages"][0]["content"])
                    self.assertNotIn("- eeAt: location of the end effector.", payload["messages"][0]["content"])
                    self.assertEqual(payload["format"], make_schema(mode))

    def test_invalid_predicate_files_fail_before_model_call(self):
        with tempfile.TemporaryDirectory() as directory:
            empty = Path(directory) / "empty.txt"
            empty.write_text(" \n")
            missing = Path(directory) / "missing.txt"
            with patch("ollama_client.urllib.request.urlopen") as request:
                for path, error in ((empty, ValueError), (missing, ValueError)):
                    with self.subTest(path=path), self.assertRaises(error):
                        run_pipeline(RESOURCE / "example_problem_image.png",
                                     predicate_definitions_path=path)
                request.assert_not_called()

    def test_predicate_file_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            definitions_path = Path(directory) / "definitions.txt"
            definitions_path.write_text("Predicate descriptions")
            with patch("ollama_client.urllib.request.urlopen") as request:
                with self.assertRaises(ValueError):
                    run_pipeline(RESOURCE / "example_problem_image.png",
                                 predicate_definitions_path=definitions_path,
                                 pddl_output=definitions_path)
                request.assert_not_called()
            self.assertEqual(definitions_path.read_text(), "Predicate descriptions")

    def test_default_description_files(self):
        self.assertEqual(DEFAULT_PREDICATE_DESC_PATH, PROJECT_ROOT / "descriptions" / "predicates.txt")
        self.assertEqual(DEFAULT_SCENE_DESC_PATH, PROJECT_ROOT / "descriptions" / "scene.txt")
        self.assertEqual(DEFAULT_GOAL_DESC_PATH, PROJECT_ROOT / "descriptions" / "goal.txt")
        prompt = build_prompt(DEFAULT_DOMAIN_PATH.read_text(encoding="utf-8"))
        for path in (DEFAULT_PREDICATE_DESC_PATH, DEFAULT_SCENE_DESC_PATH, DEFAULT_GOAL_DESC_PATH):
            self.assertIn(path.read_text(encoding="utf-8").strip(), prompt)

    def test_invalid_scene_and_goal_files_fail_before_model_call(self):
        with tempfile.TemporaryDirectory() as directory:
            empty = Path(directory) / "empty.txt"
            empty.write_text(" \n")
            missing = Path(directory) / "missing.txt"
            with patch("ollama_client.urllib.request.urlopen") as request:
                for keyword in ("scene_path", "goal_path"):
                    for path in (empty, missing):
                        with self.subTest(keyword=keyword, path=path), self.assertRaises(ValueError):
                            run_pipeline(RESOURCE / "example_problem_image.png", **{keyword: path})
                request.assert_not_called()

    def test_scene_and_goal_files_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            for keyword in ("scene_path", "goal_path"):
                path = Path(directory) / f"{keyword}.txt"
                path.write_text("Original description")
                with patch("ollama_client.urllib.request.urlopen") as request:
                    for output_keyword in ("json_output", "pddl_output"):
                        with self.subTest(keyword=keyword, output=output_keyword), self.assertRaises(ValueError):
                            run_pipeline(RESOURCE / "example_problem_image.png",
                                         **{keyword: path, output_keyword: path})
                    request.assert_not_called()
                self.assertEqual(path.read_text(), "Original description")


if __name__ == "__main__":
    unittest.main()
