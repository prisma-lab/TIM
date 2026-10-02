"""Ground an image and publish validated PDDL directly to the planner."""
import json
import math
from pathlib import Path
import time
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from task_planner_msgs.msg import PlanningRequest

from vlm_grounder.src.pddl_context import context_from_pddl
from vlm_grounder.src.ollama_client import DEFAULT_ENDPOINT, DEFAULT_MODEL
from vlm_grounder.src.problem_json_to_pddl import problem_to_pddl
from vlm_grounder.src.problem_schema import get_vocabulary, validate_context, validate_mode
from vlm_grounder.src.vlm_generate_problem import generate_problem
from task_planner.pipeline_worker import PipelineWorker, spin_node


def ground_image(image_path, settings):
    """Return both PDDL texts; files are optional diagnostics, not transport."""
    domain = Path(settings['domain']).expanduser().read_text(encoding='utf-8')
    vocabulary = get_vocabulary(domain)
    mode = settings['mode']
    validate_mode(mode)
    context = None
    context_path = settings['context']
    if mode == 'init-only':
        if not context_path:
            raise ValueError('init-only mode requires a context JSON or PDDL path')
        path = Path(context_path).expanduser()
        text = path.read_text(encoding='utf-8')
        context = (context_from_pddl(text, vocabulary.name)
                   if path.suffix.lower() == '.pddl' else json.loads(text))
        validate_context(context, vocabulary)
    elif context_path:
        raise ValueError('context is only supported in init-only mode')
    timeout = settings['vlm_timeout']
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('vlm_timeout must be positive and finite')
    problem, _ = generate_problem(
        Path(image_path).expanduser(), domain,
        scene_path=Path(settings['scene']).expanduser(),
        goal_path=Path(settings['goal']).expanduser(),
        predicate_definitions_path=Path(settings['predicate_definitions']).expanduser(),
        model=settings['model'], endpoint=settings['endpoint'], timeout=timeout,
        mode=mode, context=context)
    pddl = problem_to_pddl(problem, vocabulary.name, domain=vocabulary)
    if settings['output_directory']:
        output = Path(settings['output_directory']).expanduser()
        inputs = {Path(settings[name]).expanduser().resolve() for name in
                  ('domain', 'scene', 'goal', 'predicate_definitions')}
        inputs.add(Path(image_path).expanduser().resolve())
        if context_path:
            inputs.add(Path(context_path).expanduser().resolve())
        outputs = {output / 'generated_problem.json', output / 'generated_problem.pddl'}
        if any(path.resolve() in inputs for path in outputs):
            raise ValueError('Diagnostic outputs must not overwrite inputs')
        output.mkdir(parents=True, exist_ok=True)
        (output / 'generated_problem.json').write_text(json.dumps(problem, indent=2) + '\n')
        (output / 'generated_problem.pddl').write_text(pddl)
    return domain, pddl


class VLMGrounder(PipelineWorker):
    def __init__(self):
        super().__init__('vlm_grounder', '/vlm_grounder/status')
        share = Path(get_package_share_directory('task_planner'))
        descriptions = share / 'resource/descriptions'
        defaults = {
            'domain': str(share / 'pddl/serdar_two_objects_domain.pddl'),
            'scene': str(descriptions / 'scene.txt'),
            'goal': str(descriptions / 'goal.txt'),
            'predicate_definitions': str(descriptions / 'predicates.txt'),
            'model': DEFAULT_MODEL, 'endpoint': DEFAULT_ENDPOINT,
            'vlm_timeout': 600.0, 'mode': 'full', 'context': '',
            'image': '', 'autostart': True, 'output_directory': '',
            'planning_topic': '/two_objects/planning_request',
            'discovery_timeout': 15.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.publisher = self.create_publisher(
            PlanningRequest, self.get_parameter('planning_topic').value, 10)
        self.subscription = self.create_subscription(
            String, '/vlm_grounder/image_path', self.image_callback, 10)
        self._startup_image = (self.get_parameter('image').value
                               if self.get_parameter('autostart').value else '')
        discovery_timeout = self.get_parameter('discovery_timeout').value
        if not math.isfinite(discovery_timeout) or discovery_timeout <= 0:
            raise ValueError('discovery_timeout must be positive and finite')
        self._startup_deadline = time.monotonic() + discovery_timeout
        self._startup_timer = self.create_timer(0.1, self._startup)

    def _startup(self):
        if not self._startup_image:
            self._startup_timer.cancel()
            return
        if self.publisher.get_subscription_count() > 0:
            self._startup_timer.cancel()
            self.image_callback(String(data=self._startup_image))
        elif time.monotonic() >= self._startup_deadline:
            self._startup_timer.cancel()
            self.status('failed', str(uuid4()), detail='Planner subscriber was not discovered; resend the image path when ready.')

    def image_callback(self, msg):
        request_id = str(uuid4())
        if not msg.data.strip():
            self.status('rejected', request_id, detail='Image path is empty')
            return
        if self.publisher.get_subscription_count() == 0:
            self.status('rejected', request_id, detail='No planner subscriber; start the planner before requesting grounding')
            return
        names = ('domain', 'scene', 'goal', 'predicate_definitions', 'model', 'endpoint',
                 'vlm_timeout', 'mode', 'context', 'output_directory')
        settings = {name: self.get_parameter(name).value for name in names}
        if self.start_work(request_id, ground_image, msg.data, settings):
            # An explicit request also consumes the optional startup image.
            self._startup_timer.cancel()

    def finish_work(self, request_id, result):
        if self.publisher.get_subscription_count() == 0:
            raise RuntimeError('Planner disconnected during grounding; no request published')
        domain, problem = result
        request = PlanningRequest()
        request.header.stamp = self.get_clock().now().to_msg()
        # PDDL is not spatial data: frame_id carries correlation for this pipeline.
        request.header.frame_id = request_id
        request.domain, request.problem = domain, problem
        self.publisher.publish(request)
        self.status('published', request_id)


def main(args=None):
    spin_node(VLMGrounder, args)
