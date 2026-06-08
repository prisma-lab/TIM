#!/usr/bin/env python3
"""
VLM Task Planner Node
---------------------
Loads domain.pddl from disk, sends init/goal images + domain to Gemini Flash
to ground the scene and generate a problem.pddl, then publishes both to
/planning_request (task_planner_msgs/msg/PlanningRequest).
"""

import os
import sys
import base64
import json

import rclpy
from rclpy.node import Node
from task_planner_msgs.msg import PlanningRequest

try:
    import google.generativeai as genai
except ImportError:
    print("ERROR: google-generativeai package not found. Install with:")
    print("  pip install google-generativeai")
    sys.exit(1)


class VLMTaskPlannerNode(Node):
    def __init__(self):
        super().__init__("vlm_task_planner_node")

        # ── ROS Parameters ──────────────────────────────────────────────
        self.declare_parameter("domain_path", "domain.pddl")
        self.declare_parameter("init_image_path", "init.jpeg")
        self.declare_parameter("goal_image_path", "goal.jpeg")
        self.declare_parameter("gemini_api_key", "YOUR_GEMINI_API_KEY")
        self.declare_parameter("gemini_model", "gemini-2.0-flash")
        self.declare_parameter("topic_name", "/planning_request")

        self.domain_path = self.get_parameter("domain_path").value
        self.init_image_path = self.get_parameter("init_image_path").value
        self.goal_image_path = self.get_parameter("goal_image_path").value
        self.api_key = self.get_parameter("gemini_api_key").value
        self.model_name = self.get_parameter("gemini_model").value
        topic = self.get_parameter("topic_name").value

        # ── Publisher ───────────────────────────────────────────────────
        self.publisher_ = self.create_publisher(PlanningRequest, topic, 10)

        self.get_logger().info("VLM Task Planner Node started")
        self.get_logger().info(f"  domain  : {self.domain_path}")
        self.get_logger().info(f"  init img: {self.init_image_path}")
        self.get_logger().info(f"  goal img: {self.goal_image_path}")
        self.get_logger().info(f"  model   : {self.model_name}")

        # Run the pipeline once then stay alive for latched subscribers
        self.timer = self.create_timer(1.0, self._run_once)

    # ─────────────────────────────────────────────────────────────────
    def _run_once(self):
        """Execute the full pipeline exactly once."""
        self.timer.cancel()  # one-shot

        try:
            domain_str = self._load_domain()
            problem_str = self._call_gemini(domain_str)
            self._publish(domain_str, problem_str)
        except Exception as e:
            self.get_logger().error(f"Pipeline failed: {e}")
            raise

    # ─────────────────────────────────────────────────────────────────
    def _load_domain(self) -> str:
        path = self.domain_path
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Domain file not found: {path}")
        with open(path, "r") as f:
            domain = f.read()
        self.get_logger().info(f"Loaded domain ({len(domain)} chars)")
        return domain

    # ─────────────────────────────────────────────────────────────────
    def _load_image(self, path: str) -> dict:
        """Load an image and return a Gemini-compatible Part dict."""
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Image not found: {path}")
        with open(path, "rb") as f:
            data = f.read()

        ext = os.path.splitext(path)[1].lower()
        mime_map = {".jpeg": "image/jpeg", ".jpg": "image/jpeg", ".png": "image/png"}
        mime = mime_map.get(ext, "image/jpeg")

        self.get_logger().info(f"Loaded image {path} ({len(data)} bytes, {mime})")
        return {"mime_type": mime, "data": data}

    # ─────────────────────────────────────────────────────────────────
    def _call_gemini(self, domain_str: str) -> str:
        """Send images + domain to Gemini Flash and return problem.pddl."""
        genai.configure(api_key=self.api_key)
        model = genai.GenerativeModel(self.model_name)

        init_img = self._load_image(self.init_image_path)
        goal_img = self._load_image(self.goal_image_path)

        prompt_text = (
            "Create a problem.pddl file for given initial and goal states' images "
            "using the domain.\n"
            'There is only the black "clip" object in the photos. The other parts '
            "are aluminum profiles and belong to the environment.\n\n"
            "--- domain.pddl ---\n"
            f"{domain_str}\n"
            "--- end domain.pddl ---\n\n"
            "Return ONLY the problem.pddl content (the raw PDDL text starting with "
            "'(define (problem ...') and nothing else. No markdown fences."
        )

        self.get_logger().info("Calling Gemini Flash …")

        response = model.generate_content(
            [
                prompt_text,
                init_img,
                goal_img,
            ]
        )

        problem_str = response.text.strip()

        # Strip markdown fences if the model wraps them anyway
        if problem_str.startswith("```"):
            lines = problem_str.split("\n")
            # Remove first and last fence lines
            lines = [l for l in lines if not l.strip().startswith("```")]
            problem_str = "\n".join(lines).strip()

        self.get_logger().info(
            f"Gemini returned problem.pddl ({len(problem_str)} chars)"
        )
        self.get_logger().info(f"Problem:\n{problem_str}")
        return problem_str

    # ─────────────────────────────────────────────────────────────────
    def _publish(self, domain_str: str, problem_str: str):
        msg = PlanningRequest()
        msg.domain = domain_str
        msg.problem = problem_str

        self.publisher_.publish(msg)
        self.get_logger().info("Published PlanningRequest to /planning_request")


def main(args=None):
    rclpy.init(args=args)
    node = VLMTaskPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
