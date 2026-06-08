import argparse
import csv
import gc
import heapq
import multiprocessing
import os
import signal
import subprocess
import time
from collections import Counter
import numpy as np
import torch
import yaml
from tqdm import tqdm
from models import EffectPredictor, load_ckpt
from learn_rules import tensor_to_pddl_problem
from parse import determinize_domain, extract_actions
import utils

EXECUTION_TOLERANCE = 0.05   
            
torch.manual_seed(42)
if torch.cuda.is_available():
    torch.cuda.manual_seed_all(42)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False


def load_config():
    parser = argparse.ArgumentParser("Evaluate planning methods.")
    parser.add_argument("-c", "--config", required=True, help="YAML config file")
    parser.add_argument("--mode", choices=["deterministic", "probabilistic", "continuous", "bilevel"])
    parser.add_argument("--symbolic_model", type=str)
    parser.add_argument("--dynamics_model", type=str)
    parser.add_argument("--num_objects", type=int)
    parser.add_argument("--num_tests", type=int)
    parser.add_argument("--max_actions", type=int)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--num_sampled_domains", type=int)
    parser.add_argument("--verification_threshold", type=float)
    parser.add_argument("--cts_max_expansions", type=int)
    parser.add_argument("--cts_heuristic_weight", type=float)
    parser.add_argument("--num_processes", type=int)
    parser.add_argument("--process_timeout", type=int)
    parser.add_argument("--save_images", action="store_true")
    args = parser.parse_args()

    with open(args.config, "r") as f:
        cfg = yaml.safe_load(f)

    for key in ["mode", "symbolic_model", "dynamics_model", "num_objects",
                "num_tests", "max_actions", "seed", "num_sampled_domains",
                "verification_threshold", "cts_max_expansions",
                "cts_heuristic_weight", "num_processes", "process_timeout"]:
        val = getattr(args, key, None)
        if val is not None:
            cfg[key] = val
    if args.save_images:
        cfg["save_images"] = True

    script_dir = os.path.dirname(os.path.abspath(__file__))
    for key in ["domain_path", "prob_domain_path", "downward_path"]:
        if key in cfg and not os.path.isabs(cfg[key]):
            cfg[key] = os.path.join(script_dir, cfg[key])

    cfg.setdefault("save_images", False)
    cfg.setdefault("cts_heuristic_weight", 1.5)
    cfg.setdefault("cts_max_expansions", 3000)
    cfg.setdefault("verification_threshold", 0.07)
    cfg.setdefault("num_sampled_domains", 100)
    cfg.setdefault("process_timeout", 500)
    cfg.setdefault("seed", 1)

    return cfg

def predict_next_state(current_state, action_str, model, device):

    n_objects = current_state.shape[0]
    parts = [int(x) for x in action_str.split(",")]
    from_obj, _, from_dy, to_obj, _, to_dy = parts

    action_tensor = torch.zeros(n_objects, 8, dtype=torch.float, device=device)
    action_tensor[from_obj, :4] = torch.tensor([1, 0, from_dy, 1], dtype=torch.float)
    action_tensor[to_obj, 4:] = torch.tensor([1, 0, to_dy, 1], dtype=torch.float)

    with torch.no_grad():
        _, _, predicted_effect = model(
            current_state.unsqueeze(0),
            action_tensor.unsqueeze(0),
            torch.ones(1, n_objects, device=device)
        )
    predicted_effect = predicted_effect.squeeze(0)
    delta_grasp = predicted_effect[:, :3]
    delta_release = predicted_effect[:, 3:6]

    s2 = current_state.clone()
    s2[:, :3] += delta_grasp
    release_offset = torch.tensor([0.0, to_dy * 0.075], device=device, dtype=torch.float)
    target_pos = s2[to_obj, :2] + release_offset
    translation = torch.zeros_like(delta_grasp[0])
    translation[:2] = target_pos - s2[from_obj, :2]
    s3 = s2.clone()
    s3[from_obj, :3] += translation
    s4 = s3.clone()
    s4[:, :3] += delta_release
    return s4

def verify_plan(init_state, plan_actions, model, goal_state, threshold, device):

    current_state = init_state.clone().to(device)
    for action in plan_actions:
        current_state = predict_next_state(current_state, action, model, device)
    errors = np.linalg.norm(
        current_state[:, :3].cpu().numpy() - goal_state[:, :3].numpy(), axis=1
    )
    is_verified = bool(np.all(errors < threshold))
    return current_state, is_verified

class SearchNode:
    def __init__(self, state, plan, g_cost, h_cost, weight=1.5):
        self.state = state
        self.plan = plan
        self.g = g_cost
        self.h = h_cost
        self.f = g_cost + weight * h_cost

    def __lt__(self, other):
        return self.f < other.f if self.f != other.f else self.h < other.h


def discretize_state(state, precision=3):
    rounded = torch.round(state[:, :3].cpu() * (10**precision)) / (10**precision)
    return tuple(rounded.flatten().tolist())
   

def calculate_heuristic(current_state, goal_state):

    pos_errors = torch.linalg.norm(
        current_state[:, :3] - goal_state[:, :3].to(current_state.device), dim=1
    )
    return torch.sum(pos_errors).item()


def generate_sensible_actions(current_state, goal_state, num_objects, proximity_threshold=0.05):

    device = goal_state.device
    current_state = current_state.to(device)
    dy_options = [-1, 0, 1]
    pos_errors = torch.linalg.norm(current_state[:, :2] - goal_state[:, :2], dim=1)
    misplaced = torch.where(pos_errors > proximity_threshold)[0]
    if len(misplaced) == 0:
        misplaced = range(num_objects)

    actions = set()
    for from_obj in misplaced:
        from_obj = int(from_obj)
        from_goal_pos = goal_state[from_obj, :2]
        distances = torch.linalg.norm(current_state[:, :2] - from_goal_pos, dim=1)
        closest = torch.argmin(distances).item()
        for from_dy in dy_options:
            for to_dy in dy_options:
                actions.add(f"{from_obj},0,{from_dy},{closest},0,{to_dy}")
                actions.add(f"{from_obj},0,{from_dy},{from_obj},0,{to_dy}")
    return list(actions)


def continuous_plan_search(init_state, goal_state, model, device,max_depth, max_expansions, heuristic_weight,
                           max_open_list_size=500000):
    n_objects = init_state.shape[0]
    start_h = calculate_heuristic(init_state, goal_state)
    start_node = SearchNode(init_state.to(device), [], 0, start_h, heuristic_weight)
    open_list = [start_node]
    heapq.heapify(open_list)
    closed_set = set()
    expanded = 0
    best_plan, best_h = [], start_h

    while open_list and expanded < max_expansions:
        node = heapq.heappop(open_list)

     
        state_key = discretize_state(node.state)
        if state_key in closed_set:
            continue
        closed_set.add(state_key)
        expanded += 1

        if node.h < best_h:
            best_h = node.h
            best_plan = node.plan

        if node.h < EXECUTION_TOLERANCE:
            return node.plan

        if len(node.plan) >= max_depth:
            continue

        for action in generate_sensible_actions(node.state, goal_state, n_objects):
       
            next_state = predict_next_state(node.state, action, model, device)
            child_key = discretize_state(next_state)
            if child_key in closed_set:
                continue
            new_plan = node.plan + [action]
            new_h = calculate_heuristic(next_state, goal_state)
            if len(open_list) < max_open_list_size:
                heapq.heappush(open_list, SearchNode(
                    next_state, new_plan, len(new_plan), new_h, heuristic_weight
                ))

    return best_plan

def parse_plan_file(plan_path):

    if not os.path.exists(plan_path):
        return []
    with open(plan_path, "r") as f:
        lines = [x.strip("()\n") for x in f.readlines()][:-1] 
    if not lines:
        return []
    actions = []
    for step in lines:
        tokens = step.split(" ")
        action_name = tokens[0]
        objs = tokens[1:]
        from_arg, from_dy, to_arg, to_dy, _, _ = action_name.split("_")
        from_idx = int(from_arg[1:])
        to_idx = int(to_arg[1:])
        actions.append(f"{objs[from_idx][-1]},0,{int(from_dy)},{objs[to_idx][-1]},0,{int(to_dy)}")
    return actions


def run_fast_downward(downward_path, domain_path, problem_path, plan_file, cwd, timeout=15):
    proc = None
    try:
        proc = subprocess.Popen(
            [downward_path, "--plan-file", plan_file,
             domain_path, problem_path, "--search", "astar(blind())"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            cwd=cwd, preexec_fn=os.setsid
        )
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        if proc:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        return False
    return os.path.exists(os.path.join(cwd, plan_file))


def sample_domains(domain_path, prob_domain_path, num_domains, out_dir):

    os.makedirs(out_dir, exist_ok=True)

    with open(prob_domain_path, "r") as f:
        prob_template = f.read().replace(":probabilistic-effects", "")
    action_blocks = extract_actions(prob_template)

    paths = []
    start_idx = 0

   
    det_path = os.path.join(out_dir, "domain_determinized_0.pddl")
    with open(domain_path, "r") as f_in:
        with open(det_path, "w") as f_out:
            f_out.write(f_in.read())
    paths.append(os.path.abspath(det_path))
    start_idx = 1

    for i in range(start_idx, num_domains):
        t = i - start_idx  
        content = determinize_domain(action_blocks, prob_template, seed=t + 11 * t)
        path = os.path.join(out_dir, f"domain_determinized_{i}.pddl")
        with open(path, "w") as f:
            f.write(content)
        paths.append(os.path.abspath(path))

    return paths


def generate_problem(env, n_action, seed):

    np.random.seed(seed)
    env.reset_objects()

    actions = []
    for _ in range(n_action):
        a = env.full_random_action()
        actions.append(f"{a[0]},0,{a[3]},{a[1]},0,{a[5]}")

    init_state = torch.tensor(env.state(), dtype=torch.float)
    states = [init_state]

    for action in actions:
        parts = [int(x) for x in action.split(",")]
        env.step(parts[0], parts[3], parts[1], parts[2], parts[4], parts[5], 1, 1)
        states.append(torch.tensor(env.state(), dtype=torch.float))

    goal_state = states[-1]
    goal_graph = torch.tensor(env.get_contact_graph(), dtype=torch.long)
    return init_state, goal_state, actions, states, goal_graph


def execute_plan(env, plan_actions, save_images=False):
    images = []
    for action in plan_actions:
        parts = [int(x) for x in action.split(",")]
        if save_images:
            _, _, imgs = env.step(parts[0], parts[3], parts[1], parts[2],
                                  parts[4], parts[5], 1, 1, get_images=True)
            images.append(imgs)
        else:
            env.step(parts[0], parts[3], parts[1], parts[2],
                     parts[4], parts[5], 1, 1, get_images=False)
    final_pos = torch.tensor(env.state()[:, :3], dtype=torch.float)
    return final_pos, images


def check_success(final_pos, goal_state):
    errors = np.linalg.norm(final_pos.numpy() - goal_state[:, :3].numpy(), axis=1)
    return bool(np.all(errors < EXECUTION_TOLERANCE))

def run_single_test(test_idx, n_action, cfg, pregenerated_domains, result_queue):
    import environment 

    mode = cfg["mode"]
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    seed = cfg["seed"] + (n_action - 1) * cfg["num_tests"] + test_idx

    if mode != "continuous":
        sym_model, _ = load_ckpt(cfg["symbolic_model"], tag="best")
        sym_model.freeze()
        sym_model.to(device)

    dyn_model, _ = load_ckpt(cfg["dynamics_model"], tag="best")
    dyn_model.freeze()
    dyn_model.to(device)

    n_obj = cfg["num_objects"]
    env = environment.BlocksWorld_v4(gui=0, min_objects=n_obj, max_objects=n_obj)

    init_state, goal_state, _, states, goal_graph = generate_problem(env, n_action, seed)

    out_dir = cfg["_out_path"]
    test_dir = os.path.join(out_dir, str(n_action), str(test_idx))
    os.makedirs(test_dir, exist_ok=True)

    result = {"mode": mode}

    if mode == "deterministic":
        result.update(_run_symbolic_test(
            env, init_state, goal_state, goal_graph, sym_model, dyn_model,
            cfg["domain_path"], None, cfg, test_dir, seed, device,
            probabilistic=False
        ))
    elif mode == "probabilistic":
        result.update(_run_symbolic_test(
            env, init_state, goal_state, goal_graph, sym_model, dyn_model,
            cfg["domain_path"], pregenerated_domains, cfg, test_dir, seed, device,
            probabilistic=True
        ))
    elif mode == "continuous":
        np.random.seed(seed)
        env.reset_objects()
        max_depth = cfg["max_actions"] + 5
        plan = continuous_plan_search(
            init_state, goal_state, dyn_model, device,
            max_depth=max_depth,
            max_expansions=cfg["cts_max_expansions"],
            heuristic_weight=cfg["cts_heuristic_weight"]
        )
        final_pos, _ = execute_plan(env, plan, cfg["save_images"])
        result["success"] = check_success(final_pos, goal_state)

    elif mode == "bilevel":
        l1_result = _run_symbolic_test(
            env, init_state, goal_state, goal_graph, sym_model, dyn_model,
            cfg["domain_path"], pregenerated_domains, cfg, test_dir, seed, device,
            probabilistic=True, bilevel=True
        )
        result["l1_verified_success"] = l1_result.get("verified_success", 0)
        result["l1_verified_fail"] = l1_result.get("verified_fail", 0)
        result["l1_solved"] = l1_result.get("l1_solved", False)
        result["l2_cts_success"] = 0
        result["l2_cts_fail"] = 0
        if not l1_result.get("l1_solved", False):
            np.random.seed(seed)
            env.reset_objects()
            max_depth = cfg["max_actions"] + 5
            plan = continuous_plan_search(
                init_state, goal_state, dyn_model, device,
                max_depth=max_depth,
                max_expansions=cfg["cts_max_expansions"],
                heuristic_weight=cfg["cts_heuristic_weight"]
            )
            final_pos, _ = execute_plan(env, plan, cfg["save_images"])
            if check_success(final_pos, goal_state):
                result["l2_cts_success"] = 1
            else:
                result["l2_cts_fail"] = 1

    
    del env
    gc.collect()
    torch.cuda.empty_cache()
    result_queue.put(result)


def _run_symbolic_test(env, init_state, goal_state, goal_graph, sym_model, dyn_model,
                       domain_path, pregenerated_domains, cfg, test_dir, seed,
                       device, probabilistic=False, bilevel=False):

    result = {
        "verified_success": 0, "verified_fail": 0,
        "unverified_success": 0, "unverified_fail": 0,
        "l1_solved": False,
    }
    threshold = cfg["verification_threshold"]

    np.random.seed(seed)
    env.reset_objects()
    z, r, zn, rn = utils.state_to_problem(init_state, goal_state, sym_model)
    problem_str = tensor_to_pddl_problem(z, r, zn, rn, goal_graph, goal_graph)
    problem_path = os.path.join(test_dir, "problem.pddl")
    with open(problem_path, "w") as f:
        f.write(problem_str)

    if not probabilistic:
     
        plan_file = "sas_plan"
        run_fast_downward(cfg["downward_path"], domain_path, "problem.pddl",
                          plan_file, test_dir)
        plan_actions = parse_plan_file(os.path.join(test_dir, plan_file))

        _, is_verified = verify_plan(init_state, plan_actions, dyn_model,
                                     goal_state, threshold, device)
        np.random.seed(seed)
        env.reset_objects()
        final_pos, _ = execute_plan(env, plan_actions, cfg["save_images"])
        succeeded = check_success(final_pos, goal_state)

        if is_verified:
            if succeeded:
                result["verified_success"] = 1
            else:
                result["verified_fail"] = 1
        else:
            if succeeded:
                result["unverified_success"] = 1
            else:
                result["unverified_fail"] = 1

    else:
        prob_dir = os.path.join(test_dir, "probabilistic_planning")
        os.makedirs(prob_dir, exist_ok=True)

        plan_counter = Counter()
        plan_storage = {}

        for t, dom_path in enumerate(pregenerated_domains):
            plan_file = f"sas_plan_{t}"
            plan_output = os.path.join(prob_dir, plan_file)
            if os.path.exists(plan_output):
                os.remove(plan_output)

            run_fast_downward(cfg["downward_path"], dom_path,
                              os.path.join("..", "problem.pddl"),
                              plan_file, prob_dir)

            plan_actions = parse_plan_file(plan_output)
            plan_tuple = tuple(plan_actions)
            plan_counter[plan_tuple] += 1
            if plan_tuple not in plan_storage:
                plan_storage[plan_tuple] = plan_actions

        selected_plan = None
        selected_verified = False

        if bilevel:
            for plan_tuple, _ in plan_counter.most_common():
                plan_actions = plan_storage[plan_tuple]
                _, is_verified = verify_plan(init_state, plan_actions, dyn_model,
                                             goal_state, threshold, device)
                if is_verified:
                    selected_plan = plan_actions
                    selected_verified = True
                    break
        else:
            most_frequent_plan = None
            for plan_tuple, _ in plan_counter.most_common():
                plan_actions = plan_storage[plan_tuple]
                if most_frequent_plan is None:
                    most_frequent_plan = plan_actions
                _, is_verified = verify_plan(init_state, plan_actions, dyn_model,
                                             goal_state, threshold, device)
                if is_verified:
                    selected_plan = plan_actions
                    selected_verified = True
                    break
            if selected_plan is None:
                selected_plan = most_frequent_plan

        if selected_plan is not None:
            np.random.seed(seed)
            env.reset_objects()
            final_pos, _ = execute_plan(env, selected_plan, cfg["save_images"])
            succeeded = check_success(final_pos, goal_state)
        else:
            succeeded = False

        if bilevel:
            if selected_verified:
                if succeeded:
                    result["verified_success"] = 1
                    result["l1_solved"] = True
                else:
                    result["verified_fail"] = 1
                    result["l1_solved"] = True
        else:
            if selected_verified:
                if succeeded:
                    result["verified_success"] = 1
                else:
                    result["verified_fail"] = 1
            else:
                if succeeded:
                    result["unverified_success"] = 1
                else:
                    result["unverified_fail"] = 1

    return result


def _safe_div(a, b):
    return a / b if b else 0.0


def aggregate_results(all_results, mode, max_actions, num_tests):
    stats = {}
    for k in range(1, max_actions + 1):
        results_k = [r for r in all_results if r["n_action"] == k]
        if mode in ("deterministic", "probabilistic"):
            stats[k] = {
                "verified_success": sum(r["verified_success"] for r in results_k),
                "verified_fail": sum(r["verified_fail"] for r in results_k),
                "unverified_success": sum(r["unverified_success"] for r in results_k),
                "unverified_fail": sum(r["unverified_fail"] for r in results_k),
            }
            s = stats[k]
            s["total_success"] = s["verified_success"] + s["unverified_success"]
            s["success_rate"] = _safe_div(s["total_success"], num_tests)
        elif mode == "continuous":
            succ = sum(1 for r in results_k if r["success"])
            stats[k] = {
                "success": succ,
                "fail": len(results_k) - succ,
                "success_rate": _safe_div(succ, num_tests),
            }
        elif mode == "bilevel":
            stats[k] = {
                "l1_verified_success": sum(r["l1_verified_success"] for r in results_k),
                "l1_verified_fail": sum(r["l1_verified_fail"] for r in results_k),
                "l2_cts_success": sum(r["l2_cts_success"] for r in results_k),
                "l2_cts_fail": sum(r["l2_cts_fail"] for r in results_k),
            }
            s = stats[k]
            s["total_success"] = s["l1_verified_success"] + s["l2_cts_success"]
            s["bilevel_rate"] = _safe_div(s["total_success"], num_tests)
    return stats


def render_table(stats, mode, num_tests, max_actions):
    lines = []
    sep = "─" * 80

    if mode in ("deterministic", "probabilistic"):
        lines.append(sep)
        lines.append(f"{'k':>3} │ {'Verified':>18} │ {'Unverified':>18} │ {'Rate':>8}")
        lines.append(f"{'':>3} │ {'succ/fail':>18} │ {'succ/fail':>18} │ {'':>8}")
        lines.append(sep)
        for k in range(1, max_actions + 1):
            s = stats[k]
            v_str = f"{s['verified_success']:>3}/{s['verified_fail']:<3}"
            u_str = f"{s['unverified_success']:>3}/{s['unverified_fail']:<3}"
            lines.append(
                f"{k:>3} │ {v_str:>18} │ {u_str:>18} │ "
                f"{s['success_rate']*100:>6.1f}%"
            )
    elif mode == "continuous":
        lines.append(sep)
        lines.append(f"{'k':>3} │ {'Success':>8} │ {'Fail':>8} │ {'Rate':>8}")
        lines.append(sep)
        for k in range(1, max_actions + 1):
            s = stats[k]
            lines.append(f"{k:>3} │ {s['success']:>8} │ {s['fail']:>8} │ {s['success_rate']*100:>6.1f}%")
    elif mode == "bilevel":
        lines.append(sep)
        lines.append(f"{'k':>3} │ {'L1 Verified':>18} │ {'L2 CTS':>18} │ {'Bilevel Rate':>12}")
        lines.append(f"{'':>3} │ {'succ/fail':>18} │ {'succ/fail':>18} │ {'':>12}")
        lines.append(sep)
        for k in range(1, max_actions + 1):
            s = stats[k]
            l1_str = f"{s['l1_verified_success']:>3}/{s['l1_verified_fail']:<3}"
            l2_str = f"{s['l2_cts_success']:>3}/{s['l2_cts_fail']:<3}"
            lines.append(
                f"{k:>3} │ {l1_str:>18} │ {l2_str:>18} │ {s['bilevel_rate']*100:>10.1f}%"
            )

    lines.append(sep)
    return "\n".join(lines)


def save_results(stats, out_dir, mode, num_tests, max_actions):
    os.makedirs(out_dir, exist_ok=True)
    table = render_table(stats, mode, num_tests, max_actions)

    txt_path = os.path.join(out_dir, f"results_{mode}.txt")
    with open(txt_path, "w") as f:
        f.write(table + "\n")

    csv_path = os.path.join(out_dir, f"results_{mode}.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        if mode in ("deterministic", "probabilistic"):
            w.writerow(["k", "verified_success", "verified_fail",
                        "unverified_success", "unverified_fail",
                        "total_success", "success_rate"])
            for k in range(1, max_actions + 1):
                s = stats[k]
                w.writerow([k, s["verified_success"], s["verified_fail"],
                            s["unverified_success"], s["unverified_fail"],
                            s["total_success"],
                            f"{s['success_rate']:.4f}"])
        elif mode == "continuous":
            w.writerow(["k", "success", "fail", "success_rate"])
            for k in range(1, max_actions + 1):
                s = stats[k]
                w.writerow([k, s["success"], s["fail"], f"{s['success_rate']:.4f}"])
        elif mode == "bilevel":
            w.writerow(["k", "l1_verified_success", "l1_verified_fail",
                        "l2_cts_success", "l2_cts_fail",
                        "total_success", "bilevel_rate"])
            for k in range(1, max_actions + 1):
                s = stats[k]
                w.writerow([k, s["l1_verified_success"], s["l1_verified_fail"],
                            s["l2_cts_success"], s["l2_cts_fail"],
                            s["total_success"], f"{s['bilevel_rate']:.4f}"])

    return table, txt_path, csv_path


def main():
    cfg = load_config()
    mode = cfg["mode"]
    num_tests = cfg["num_tests"]
    max_actions = cfg["max_actions"]
    num_processes = cfg["num_processes"]

    out_path = os.path.join("out", mode, cfg.get("symbolic_model", cfg["dynamics_model"]),
                            str(cfg["num_objects"]))
    if os.path.exists(out_path):
        out_path = out_path + f"_{int(time.time())}"
    os.makedirs(out_path, exist_ok=True)
    cfg["_out_path"] = out_path

    with open(os.path.join(out_path, "config.yaml"), "w") as f:
        yaml.dump({k: v for k, v in cfg.items() if not k.startswith("_")}, f)

    pregenerated_domains = []
    if mode in ("probabilistic", "bilevel"):
        domain_dir = os.path.join(out_path, "sampled_domains")
        pregenerated_domains = sample_domains(
            cfg["domain_path"], cfg["prob_domain_path"],
            cfg["num_sampled_domains"],
            domain_dir
        )
        print(f"Pre-generated {len(pregenerated_domains)} domains in {domain_dir}")

    all_results = []
    for n_action in range(1, max_actions + 1):
        print(f"\n{'='*60}")
        print(f"Running {num_tests} tests for k={n_action} ({mode} mode, "
              f"{cfg['num_objects']} objects, {num_processes} processes)")
        print(f"{'='*60}")

        tasks = list(range(num_tests))
        active = {}

        with tqdm(total=num_tests, desc=f"k={n_action}") as pbar:
            while tasks or active:
                while tasks and len(active) < num_processes:
                    idx = tasks.pop(0)
                    q = multiprocessing.Queue()
                    p = multiprocessing.Process(
                        target=run_single_test,
                        args=(idx, n_action, cfg, pregenerated_domains, q)
                    )
                    p.start()
                    active[p] = (idx, q, time.time())

                for p in list(active):
                    idx, q, start = active[p]
                    p.join(timeout=1)

                    if not p.is_alive():
                        try:
                            r = q.get_nowait()
                            r["n_action"] = n_action
                            r["test_idx"] = idx
                            all_results.append(r)
                        except Exception:
                            pass
                        del active[p]
                        pbar.update(1)
                    elif time.time() - start > cfg["process_timeout"]:
                        tqdm.write(f"Worker {idx} timed out, terminating.")
                        p.terminate()
                        p.join()
                        del active[p]
                        pbar.update(1)

                time.sleep(0.1)

    stats = aggregate_results(all_results, mode, max_actions, num_tests)
    table, txt_path, csv_path = save_results(stats, out_path, mode, num_tests, max_actions)
    print(f"\n{table}")
    print(f"\nResults saved to: {txt_path}")
    print(f"CSV saved to:     {csv_path}")


if __name__ == "__main__":
    multiprocessing.set_start_method("spawn")
    t0 = time.time()
    main()
    print(f"\nTotal time: {time.time() - t0:.1f}s")