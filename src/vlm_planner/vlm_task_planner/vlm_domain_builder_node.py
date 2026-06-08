#!/usr/bin/env python3
"""
VLM Domain Builder Node
-----------------------
Implements the *domain-construction* phase of the VLM symbolic-domain-extraction
pipeline. It reads a dataset of action snapshots -- each a triple
(pre-action image, post-action image, action label) -- prompts a VLM (Gemini) to
synthesise a single PDDL domain that explains all snapshots, writes the result to
``output_domain_path`` and publishes the domain string on ``/generated_domain``
(std_msgs/String).

The produced ``domain.pddl`` is the artefact consumed by the inference node
(``vlm_task_planner_node``), which loads it from disk to ground problems and
request plans. The two nodes therefore compose through the domain file.

Integration notes (this is a prototype -- "no need to be perfect"):
  * Add to setup.py console_scripts:
        "vlm_domain_builder_node = vlm_task_planner.vlm_domain_builder_node:main",
  * Add to package.xml:
        <exec_depend>std_msgs</exec_depend>
  * Provide a manifest.json next to the snapshot images, e.g.:
        [
          {"label": "slide",    "pre": "slide_0_pre.jpg",    "post": "slide_0_post.jpg"},
          {"label": "slide",    "pre": "slide_1_pre.jpg",    "post": "slide_1_post.jpg"},
          {"label": "pick-up",  "pre": "pick_0_pre.jpg",     "post": "pick_0_post.jpg"},
          {"label": "transfer", "pre": "transfer_0_pre.jpg", "post": "transfer_0_post.jpg"},
          {"label": "release",  "pre": "release_0_pre.jpg",  "post": "release_0_post.jpg"}
        ]
    Duplicate labels are allowed and act as extra evidence for the same operator.
  * Run:
        ros2 run vlm_task_planner vlm_domain_builder_node \
            --ros-args -p manifest_path:=/abs/snapshots/manifest.json \
                       -p snapshots_dir:=/abs/snapshots \
                       -p output_domain_path:=/abs/domain.pddl \
                       -p gemini_api_key:=AIza...
"""

import os
import sys
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import google.generativeai as genai
except ImportError:
    print("ERROR: google-generativeai package not found. Install with:")
    print("  pip install google-generativeai")
    sys.exit(1)


# The textual part of the multimodal prompt. The (label, pre, post) snapshots are
# appended after this text, interleaved, in _call_gemini().
DOMAIN_PROMPT = """You are given a set of action snapshots. Each snapshot is a
triple (pre-action image, post-action image, action label). Snapshots that share
the same label are different instances of the same action.

Construct a SINGLE PDDL domain that explains all snapshots:
  - Define a shared set of typed predicates sufficient to describe the object and
    robot states that are visible in the images.
  - For each distinct action label, define exactly one lifted operator
    (:action <label>) with typed :parameters, a :precondition that holds in the
    pre-action images, and an :effect (add/delete) that reflects the change
    observed between the pre- and post-action images.
  - Reuse the same predicates consistently across operators.

Return ONLY the domain.pddl content, starting with '(define (domain '.
No markdown fences and no commentary.
"""


class VLMDomainBuilderNode(Node):
    def __init__(self):
        super().__init__("vlm_domain_builder_node")

        # ── ROS Parameters ──────────────────────────────────────────────
        self.declare_parameter("snapshots_dir", "snapshots")
        self.declare_parameter("manifest_path", "snapshots/manifest.json")
        self.declare_parameter("output_domain_path", "domain.pddl")
        self.declare_parameter("gemini_api_key", "YOUR_GEMINI_API_KEY")
        self.declare_parameter("gemini_model", "gemini-2.0-flash")
        self.declare_parameter("topic_name", "/generated_domain")

        self.snapshots_dir = self.get_parameter("snapshots_dir").value
        self.manifest_path = self.get_parameter("manifest_path").value
        self.output_domain_path = self.get_parameter("output_domain_path").value
        self.api_key = self.get_parameter("gemini_api_key").value
        self.model_name = self.get_parameter("gemini_model").value
        topic = self.get_parameter("topic_name").value

        # ── Publisher ───────────────────────────────────────────────────
        self.publisher_ = self.create_publisher(String, topic, 10)

        self.get_logger().info("VLM Domain Builder Node started")
        self.get_logger().info(f"  manifest  : {self.manifest_path}")
        self.get_logger().info(f"  out domain: {self.output_domain_path}")
        self.get_logger().info(f"  model     : {self.model_name}")

        # Run the pipeline once, then stay alive for latched subscribers.
        self.timer = self.create_timer(1.0, self._run_once)

    # ─────────────────────────────────────────────────────────────────
    def _run_once(self):
        """Execute the domain-construction phase exactly once."""
        self.timer.cancel()  # one-shot
        try:
            snapshots = self._load_manifest()
            domain_str = self._call_gemini(snapshots)
            self._write_domain(domain_str)
            self._publish(domain_str)
        except Exception as e:
            self.get_logger().error(f"Domain construction failed: {e}")
            raise

    # ─────────────────────────────────────────────────────────────────
    def _load_manifest(self) -> list:
        """Return a list of {'label', 'pre', 'post'} snapshot entries."""
        if not os.path.isfile(self.manifest_path):
            raise FileNotFoundError(f"Manifest not found: {self.manifest_path}")
        with open(self.manifest_path, "r") as f:
            entries = json.load(f)
        if not entries:
            raise ValueError("Manifest is empty")
        labels = sorted({e["label"] for e in entries})
        self.get_logger().info(
            f"Loaded {len(entries)} snapshots over {len(labels)} action labels: {labels}"
        )
        return entries

    # ─────────────────────────────────────────────────────────────────
    def _load_image(self, fname: str) -> dict:
        """Load an image and return a Gemini-compatible Part dict."""
        path = fname if os.path.isabs(fname) else os.path.join(self.snapshots_dir, fname)
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Image not found: {path}")
        with open(path, "rb") as f:
            data = f.read()
        ext = os.path.splitext(path)[1].lower()
        mime_map = {".jpeg": "image/jpeg", ".jpg": "image/jpeg", ".png": "image/png"}
        mime = mime_map.get(ext, "image/jpeg")
        return {"mime_type": mime, "data": data}

    # ─────────────────────────────────────────────────────────────────
    def _call_gemini(self, snapshots: list) -> str:
        """Send the prompt + interleaved snapshots to Gemini, return domain.pddl."""
        genai.configure(api_key=self.api_key)
        model = genai.GenerativeModel(self.model_name)

        # Interleave the textual prompt with each (label, pre, post) snapshot.
        parts = [DOMAIN_PROMPT]
        for i, e in enumerate(snapshots):
            parts.append(f"\n# snapshot {i} | action label: {e['label']}\npre-action image:")
            parts.append(self._load_image(e["pre"]))
            parts.append("post-action image:")
            parts.append(self._load_image(e["post"]))

        self.get_logger().info(
            f"Calling {self.model_name} with {len(snapshots)} snapshots …"
        )
        response = model.generate_content(parts)
        domain_str = response.text.strip()

        # Strip markdown fences if the model wraps them anyway.
        if domain_str.startswith("```"):
            lines = [l for l in domain_str.split("\n") if not l.strip().startswith("```")]
            domain_str = "\n".join(lines).strip()

        self.get_logger().info(f"VLM returned domain.pddl ({len(domain_str)} chars)")
        self.get_logger().info(f"Domain:\n{domain_str}")
        return domain_str

    # ─────────────────────────────────────────────────────────────────
    def _write_domain(self, domain_str: str):
        with open(self.output_domain_path, "w") as f:
            f.write(domain_str)
        self.get_logger().info(f"Wrote domain to {self.output_domain_path}")

    # ─────────────────────────────────────────────────────────────────
    def _publish(self, domain_str: str):
        msg = String()
        msg.data = domain_str
        self.publisher_.publish(msg)
        self.get_logger().info("Published generated domain on the domain topic")


def main(args=None):
    rclpy.init(args=args)
    node = VLMDomainBuilderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
