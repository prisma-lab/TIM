"""Offline checks for typed CRF grounding and partial initial-state completion."""

import copy
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from pddl_context import context_from_pddl, parse_domain
from problem_json_to_pddl import convert_problem_file
from problem_schema import complete_init_problem, make_schema, validate_context, validate_problem
from vlm_generate_problem import main, run_pipeline


ROOT = Path(__file__).resolve().parent.parent
RESOURCE = ROOT / "resource"
CRF_DESCRIPTIONS = ROOT / "descriptions" / "crf"


class CrfPipelineTests(unittest.TestCase):
    def setUp(self):
        self.domain_path = RESOURCE / "crf_domain.pddl"
        self.domain = self.domain_path.read_text()
        self.pddl = (RESOURCE / "crf_example_problem.pddl").read_text()
        self.context = context_from_pddl(self.pddl, "battery-pack-assembly")

    def response(self, generated):
        return io.BytesIO(json.dumps({"message": {"content": json.dumps(generated)}}).encode())

    def run_crf(self, output, **kwargs):
        return run_pipeline(
            RESOURCE / "crf_problem.jpg", domain_path=self.domain_path,
            json_output=output, scene_path=CRF_DESCRIPTIONS / "scene.txt",
            goal_path=CRF_DESCRIPTIONS / "goal.txt",
            predicate_definitions_path=CRF_DESCRIPTIONS / "predicates.txt", **kwargs,
        )

    def test_complete_example_and_json_context_match(self):
        example = json.loads((RESOURCE / "context.json").read_text())
        self.assertEqual(example, self.context)
        self.assertEqual(len(example["objects"]), 14)
        self.assertEqual(len(example["init"]), 14)
        self.assertEqual(example["goal"], [
            {"predicate": "installedAt", "args": ["bar", "barLoc"]},
            {"predicate": "installedAt", "args": ["connectors", "connectorsLoc"]},
        ])
        validate_context(example, self.domain)

    def test_domain_schema_and_subtype_validation(self):
        vocabulary = parse_domain(self.domain)
        schema = make_schema(domain=self.domain)
        self.assertIn("conBar", schema["properties"]["objects"]["items"]["properties"]["type"]["enum"])
        self.assertEqual(set(schema["properties"]["init"]["items"]["properties"]["predicate"]["enum"]),
                         {"available", "free", "at", "tightened", "installedAt"})
        self.assertTrue(vocabulary.is_subtype("conBar", "part"))
        self.assertTrue(vocabulary.is_subtype("conBarLoc", "location"))
        self.assertTrue(vocabulary.is_subtype("conBar", "object"))
        self.assertFalse(vocabulary.is_subtype("conBarLoc", "part"))
        validate_problem(self.context, self.domain)
        for fact in ({"predicate": "available", "args": ["barLoc"]},
                     {"predicate": "free", "args": ["bar"]},
                     {"predicate": "at", "args": ["barLoc", "bar"]},
                     {"predicate": "tightened", "args": ["bar", "plate"]}):
            invalid = {**self.context, "init": [fact]}
            with self.subTest(fact=fact), self.assertRaises(ValueError):
                validate_problem(invalid, self.domain)

    def test_cli_completes_edited_pddl_and_preserves_retained_facts(self):
        missing = [self.context["init"][0], self.context["init"][5], self.context["init"][7]]
        partial = self.pddl
        for fact in missing:
            partial = partial.replace(f"({fact['predicate']} {' '.join(fact['args'])})", "")
        # Repeated facts, including different PDDL case, must not duplicate output.
        generated = {"init": missing + [{"predicate": "AVAILABLE", "args": ["CAPS"]}]}
        with tempfile.TemporaryDirectory() as directory:
            context_path = Path(directory) / "partial.pddl"
            context_path.write_text(partial)
            output = Path(directory) / "completed.json"
            with patch("ollama_client.urllib.request.urlopen", return_value=self.response(generated)) as request:
                main([
                    str(RESOURCE / "crf_problem.jpg"), "--domain", str(self.domain_path),
                    "--mode", "init-only", "--context", str(context_path),
                    "--scene", str(CRF_DESCRIPTIONS / "scene.txt"),
                    "--predicate-definitions", str(CRF_DESCRIPTIONS / "predicates.txt"),
                    "--goal", str(Path(directory) / "unused-goal.txt"),
                    "--problem-name", "assemble-battery-pack", "--output", str(output),
                ])
            problem = json.loads(output.read_text())
            retained = context_from_pddl(partial, "battery-pack-assembly")["init"]
            self.assertEqual(problem, {**self.context, "init": retained + missing})
            self.assertEqual(context_path.read_text(), partial)
            pddl = output.with_suffix(".pddl").read_text()
            self.assertIn("(problem assemble-battery-pack)", pddl)
            self.assertIn("bar - conBar", pddl)
            self.assertIn("screws1 screws2 - screws", pddl)
            self.assertIn("barLoc - conBarLoc", pddl)
            round_trip = context_from_pddl(pddl, "battery-pack-assembly")
            self.assertEqual({entity["name"]: entity["type"] for entity in round_trip["objects"]},
                             {entity["name"]: entity["type"] for entity in problem["objects"]})
            self.assertEqual(round_trip["init"], problem["init"])
            self.assertEqual(round_trip["goal"], problem["goal"])
            payload = json.loads(request.call_args.args[0].data)
            self.assertEqual(payload["format"], make_schema("init-only", self.domain))
            prompt = payload["messages"][0]["content"]
            self.assertIn(json.dumps(context_from_pddl(partial, "battery-pack-assembly")), prompt)
            self.assertIn("will be preserved automatically", prompt)
            self.assertIn("do not contradict", prompt)

    def test_json_partial_init_merge_is_stable_and_does_not_mutate_context(self):
        context = copy.deepcopy(self.context)
        context["init"] = context["init"][:3]
        original = copy.deepcopy(context)
        generated = {"init": [context["init"][1], {"predicate": "AVAILABLE", "args": ["BAR"]},
                              self.context["init"][3], self.context["init"][3]]}
        result = complete_init_problem(generated, context, self.domain)
        self.assertEqual(result["init"], context["init"] + [self.context["init"][3]])
        self.assertEqual(context, original)
        self.assertEqual(complete_init_problem({"init": []}, context, self.domain), context)
        without_init = {key: context[key] for key in ("objects", "goal")}
        self.assertEqual(complete_init_problem({"init": []}, without_init, self.domain)["init"], [])

    def test_json_context_file_preserves_retained_init(self):
        context = {**self.context, "init": self.context["init"][:2]}
        with tempfile.TemporaryDirectory() as directory:
            context_path = Path(directory) / "context.json"
            context_path.write_text(json.dumps(context))
            output = Path(directory) / "completed.json"
            with patch("ollama_client.urllib.request.urlopen", return_value=self.response({"init": []})):
                self.run_crf(output, mode="init-only", context_path=context_path)
            self.assertEqual(json.loads(output.read_text()), context)

    def test_merged_contradictions_preserve_existing_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "problem.json"
            pddl_output = output.with_suffix(".pddl")
            output.write_text("existing json")
            pddl_output.write_text("existing pddl")
            for predicate in ("at", "installedAt"):
                generated = {"init": [{"predicate": predicate, "args": ["bar", "barLoc"]}]}
                with self.subTest(predicate=predicate):
                    with patch("ollama_client.urllib.request.urlopen", return_value=self.response(generated)):
                        with self.assertRaisesRegex(ValueError, "occupied and free"):
                            self.run_crf(output, mode="init-only", context=self.context)
                    self.assertEqual(output.read_text(), "existing json")
                    self.assertEqual(pddl_output.read_text(), "existing pddl")

    def test_invalid_retained_facts_fail_before_model_call(self):
        for init in ("bad", [{"predicate": "free", "args": ["bar"]}],
                     [{"predicate": "available", "args": ["undeclared"]}],
                     [{"predicate": "available", "args": ["bar"], "extra": True}],
                     self.context["init"] + [{"predicate": "at", "args": ["bar", "barLoc"]}]):
            with self.subTest(init=init), patch("ollama_client.urllib.request.urlopen") as request:
                with self.assertRaises(ValueError):
                    self.run_crf(ROOT / "output" / "unused.json", mode="init-only",
                                 context={**self.context, "init": init})
                request.assert_not_called()

    def test_full_crf_generation_and_standalone_conversion(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "full.json"
            with patch("ollama_client.urllib.request.urlopen", return_value=self.response(self.context)) as request:
                self.run_crf(output)
            self.assertEqual(json.loads(output.read_text()), self.context)
            payload = json.loads(request.call_args.args[0].data)
            self.assertEqual(payload["format"], make_schema(domain=self.domain))
            converted = Path(directory) / "converted.pddl"
            convert_problem_file(output, self.domain_path, converted)
            self.assertEqual(converted.read_text(), output.with_suffix(".pddl").read_text())

    def test_pddl_context_domain_mismatch_and_unsupported_facts(self):
        invalid_pddl = [
            self.pddl.replace("battery-pack-assembly", "other-domain"),
            self.pddl[:-1],
            self.pddl.replace("(available bar)", "(not (available bar))"),
            self.pddl.replace("(available bar)", "(= (cost) 0)"),
            self.pddl.replace("(installedAt bar barLoc)", "(or (available bar) (available plate))"),
            self.pddl.replace("(:init", "(:metric minimize (cost))\n  (:init"),
        ]
        with tempfile.TemporaryDirectory() as directory:
            context_path = Path(directory) / "invalid.pddl"
            for text in invalid_pddl:
                context_path.write_text(text)
                with self.subTest(text=text), patch("ollama_client.urllib.request.urlopen") as request:
                    with self.assertRaises(ValueError):
                        self.run_crf(Path(directory) / "out.json", mode="init-only", context_path=context_path)
                    request.assert_not_called()

    def test_pddl_context_output_collision(self):
        with tempfile.TemporaryDirectory() as directory:
            context_path = Path(directory) / "partial.pddl"
            context_path.write_text(self.pddl)
            with patch("ollama_client.urllib.request.urlopen") as request:
                with self.assertRaises(ValueError):
                    self.run_crf(Path(directory) / "out.json", mode="init-only",
                                 context_path=context_path, pddl_output=context_path)
                request.assert_not_called()
            self.assertEqual(context_path.read_text(), self.pddl)

    def test_legacy_partial_init_rejects_cross_source_conflicts(self):
        context = {
            "objects": [{"name": "robot", "type": "robot"}, {"name": "box", "type": "object"}],
            "goal": [{"predicate": "gripperEmpty", "args": ["robot"]}],
            "init": [{"predicate": "gripperEmpty", "args": ["robot"]}],
        }
        generated = {"init": [{"predicate": "gripperHolding", "args": ["robot", "box"]}]}
        with self.assertRaisesRegex(ValueError, "cannot hold an object and be empty"):
            complete_init_problem(generated, context)

    def test_domain_hierarchy_errors_and_other_predicate_arities(self):
        for types in ("a - b b - a", "a - missing", "a A", "object - a a"):
            with self.subTest(types=types), self.assertRaises(ValueError):
                parse_domain(f"(define (domain broken) (:types {types}) (:predicates (ready)))")
        domain = """; Comments must not contribute declarations: (:types wrong)
        (define (domain generic)
          (:types leaf - branch branch - object)
          (:predicates (ready) (triple ?a ?b - branch ?c - object)))"""
        schema = make_schema(domain=domain)
        args = schema["properties"]["init"]["items"]["properties"]["args"]
        self.assertEqual((args["minItems"], args["maxItems"]), (0, 3))
        validate_problem({
            "objects": [{"name": "item", "type": "LEAF"}],
            "init": [{"predicate": "TRIPLE", "args": ["ITEM", "item", "item"]}],
            "goal": [{"predicate": "ready", "args": []}],
        }, domain)


if __name__ == "__main__":
    unittest.main()
