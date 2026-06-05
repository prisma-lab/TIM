from pyhop_domain import *
from CoppeliaEnv import CoppeliaEnv, UnityAgent
import numpy as np
import random

def print_fancy_grid(state):
    """Print the planning grid with object symbols and robot position."""
    rows = state.rows
    cols = state.cols

    # Column header (0 1 2 ...)
    header = "    " + "   ".join([str(c) for c in range(cols)])
    print("\n" + header)

    # Top border
    print("  +" + "---+" * cols)

    for r in range(rows):
        # Row start with row number
        line = f"{r} |"

        for c in range(cols):
            obj = state.grid[r][c]
            symbol = "   " # Default: empty cell

            # Symbol mapping logic
            if obj == "free":
                symbol = " . "
            elif obj.startswith("cylinder"):
                # Extract trailing number (e.g. cylinder1 -> C1)
                num = obj.replace("cylinder", "")
                symbol = f" C{num}"
            elif obj.startswith("bottle"):
                num = obj.replace("bottle", "")
                symbol = f" B{num}"
            elif obj.startswith("glass"):
                num = obj.replace("glass", "")
                symbol = f" G{num}"
            elif obj == "pillar":
                symbol = "###"
            else:
                # For other objects use first 2 letters
                symbol = f" {obj[:2].upper()}"

            # Mark robot position — overwrite cell symbol if robot is here
            # state.robot_at is a tuple (r, c) when on the grid
            if isinstance(state.robot_at, tuple) or isinstance(state.robot_at, list):
                if state.robot_at[0] == r and state.robot_at[1] == c:
                    # Robot on top of an object: append marker and re-center
                    symbol = symbol.replace(" ", "") + "*"
                    symbol = f"{symbol:^3}"

            line += symbol + "|"

        print(line)
        # Bottom border of row
        print("  +" + "---+" * cols)

    # Additional info below the grid
    print(f"  Robot at: {state.robot_at}")
    print(f"  Holding : {state.holding}")

max_steps = 1500
new_pos = [0, 0, 0]
rot = [0, 0, 0, 1]
grab = 0
left_pos = [0.150, 0.170, -0.140]
RECORD = False
LEFT_SIDE = [0, 0.3, 0]
RIGHT_SIDE = [0, -0.3, 0]
HOME = [0, 0, 0]

OBJ_TO_ID = {
    0: "cylinder",
    1: "cube",
    2: "bottle",
    3: "wine_glass"
}

OBJ_TO_HANDLE = {}

def init_coppelia_env(env, obstacles=0, object_to_grab=2, n_objects=2, randomize_pos=True):
    """Reset the CoppeliaSim environment and spawn objects on the table."""
    random_n_objects = True if n_objects == -1 else False
    env.reset(RECORD)
    if random_n_objects:
        n_objects = random.randint(2, 3)
    env.init_clean_and_place(obstacles=obstacles, object_to_grab=object_to_grab, n_objects=n_objects, randomize_pos=randomize_pos)


def init_planning_env(obj_grid_dict, half, n_rows=6, n_cols=6, **kwargs):
    """Build the initial PyHOP planning state from the CoppeliaSim object grid."""
    state1 = state1 = pyhop.State("state1")
    state1.locations = ["box", "waste", "A_box", "home", "left_side", "right_side"]
    state1.box_on_the_left = False if half == 0 else True
    state1.rows, state1.cols = (n_rows, n_cols)
    state1.grid = [["free" for x in range(state1.rows)] for y in range(state1.cols)] 
    state1.near = {
        "box": "A_box",
        "A_box": "box"
    }

    state1.objects = []
    state1.movable = []
    
    detected_objects = {}

    for i, coord in enumerate(obj_grid_dict):
        row, col = coord

        if obj_grid_dict[coord][1] not in detected_objects:
            detected_objects[obj_grid_dict[coord][1]] = 1
        else:
            detected_objects[obj_grid_dict[coord][1]] += 1

        OBJ_TO_HANDLE[f"{OBJ_TO_ID[obj_grid_dict[coord][1]]}{detected_objects[obj_grid_dict[coord][1]]}"] = obj_grid_dict[coord][0]

        state1.grid[row][col] = f"{OBJ_TO_ID[obj_grid_dict[coord][1]]}{detected_objects[obj_grid_dict[coord][1]]}"
        state1.objects.append(f"{OBJ_TO_ID[obj_grid_dict[coord][1]]}{detected_objects[obj_grid_dict[coord][1]]}")
        if obj_grid_dict[coord][2] == 1:
            state1.movable.append(f"{OBJ_TO_ID[obj_grid_dict[coord][1]]}{detected_objects[obj_grid_dict[coord][1]]}")
    #state1.objects = [f"{OBJ_TO_ID[obj[1]]}{i+1}" for i, obj in enumerate(obj_grid_dict.values())]

    state1.robot_at = "home"
    state1.holding = None
    state1.box = None
    return state1

def plan(start_state, tasks, verbose=0):
    """Run HTN planning and return the sequence of primitive actions."""
    return pyhop.pyhop(start_state, tasks, verbose=verbose)

def move_to(env, pos):
    """Move the end-effector to pos using the current global rotation, preserving the held object."""
    global rot
    global grab
    while not env.check_dist(pos, rot):
        displacement, delta_rotation = env.move_target(pos, rot)
        env.step(displacement, delta_rotation, grab)
        env.step_simulation()
        if grab == 1 and env.check_obj_away():
            return False
    return True

def move_to_pos_rot(env, pos, rot=None):
    """Move the end-effector to pos with an explicit target rotation."""
    global grab
    while not env.check_dist(pos, rot):
        displacement, delta_rotation = env.move_target(pos, rot)
        env.step(displacement, delta_rotation, grab)
        env.step_simulation()
        if grab == 1 and env.check_obj_away():
            return False
    return True

def pick_up(env, agent, obj, side):
    """Execute the grasp-and-lift policy until the object is lifted or max_steps is reached."""
    global new_pos
    global rot
    global grab
    global obstacle
    env.set_obj_handle(obj)
    for _ in range(max_steps):
        if env.check_fallen_object():
            return False
        obs = env.get_observations_grasp_and_lift(side)
        displacement, delta_rotation, close = agent.predict(obs, phase=0)
        env.step(displacement, delta_rotation, close)
        env.step_simulation()

        if env.start_reach_target_phase():
            rot = env.sim.getObjectQuaternion(env.target_handle)
            grab = 1
            if obstacle:
                new_pos = env.get_object_local_pos()
                new_pos[2] = 0.05
                print(new_pos)
            return True
    return False

def reach(env):
    """Move the end-effector toward the box target, stopping 0.2 m short of it."""
    global new_pos
    global rot
    for _ in range(max_steps):
        dist = np.array(env.sim.getObjectPosition(env.object_target, env.reference_frame)) - np.array(env.sim.getObjectPosition(env.obj_handle, env.reference_frame))
        dist_normalized = dist / np.linalg.norm(dist)
        new_pos = np.array(env.sim.getObjectPosition(env.target_handle, env.reference_frame)) + dist - (dist_normalized * 0.2)
        if move_to(env, new_pos):
            return True
    return False

def release_obj(env):
    """Open the gripper for 20 steps to release the held object."""
    global grab
    release_steps = 0
    for i in range(max_steps):
        env.open_gripper()
        env.step_simulation()
        if release_steps < 20:
            release_steps += 1
        else:
            grab = 0
            return True
    return False

def place_obj(env, agent):
    """Run the place policy until the object has been in the target zone for 50 steps."""
    collision_steps = 0
    for _ in range(max_steps):
        if collision_steps < 50:
            obs = env.get_observations_place()
            displacement, delta_rotation, close = agent.predict(obs, 2)
            env.step(displacement, delta_rotation, close)
            env.step_simulation()
            collision_steps += 1 if env.obj_target_collision() else 0
        else:
            return True
    return False

def parse_plan(plan, env, agent, obj_to_grab):
    """Translate the symbolic HTN plan into a list of executable (function, args) pairs."""
    global new_pos
    global obstacle
    parsed_plan = []
    for action in plan:
        actions = []
        if action[0] == "move":
            if action[2] == "left_side":
                actions.append((move_to_pos_rot, [env, LEFT_SIDE, [0, 0, 0, 1]]))
            elif action[2] == "right_side":
                actions.append((move_to_pos_rot, [env, RIGHT_SIDE, [0, 0, 0, 1]]))   
            elif action[2] == "home":
                if action[1] == "waste" or action[1] == "A_box":
                    actions.append((move_to_pos_rot, [env, HOME, [0, 0, 0, 1]]))
                else:
                    actions.append((move_to, [env, HOME]))
            elif action[2] == "A_box":
                actions.append((reach, [env]))
            elif action[2] == "waste":
                actions.append((move_to, [env, np.array(env.sim.getObjectPosition(env.drop_point, env.reference_frame)) - np.array((random.uniform(-0.02, 0.02), random.uniform(0.0, 0.066), 0))]))
            else: #tuple
                obj_pos = env.sim.getObjectPosition(env.obj_grid_dict[(action[2][0], action[2][1])][0], env.reference_frame)
                #actions.append((move_to, [env, np.array([obj_pos[0], obj_pos[1], 0])]))
                actions.append((move_to_pos_rot, [env, np.array([obj_pos[0], obj_pos[1], 0]), [0, 0, 0, 1]]))

        elif action[0] == "pickUp":
            obstacle = False if action[2] == obj_to_grab else True
            if action[1] == "side":  
                actions.append((pick_up, [env, agent, OBJ_TO_HANDLE[action[2]], 1]))
            elif action[1] == "up":
                actions.append((pick_up, [env, agent, OBJ_TO_HANDLE[action[2]], 0]))

            if obstacle:
                obj_pos = env.sim.getObjectPosition(OBJ_TO_HANDLE[action[2]], env.reference_frame)
                actions.append((move_to, [env, np.array([obj_pos[0], obj_pos[1], obj_pos[2] + 0.25])]))

        elif action[0] == "release":
            actions.append((release_obj, [env]))
        elif action[0] == "place":
            actions.append((place_obj, [env, agent]))
        parsed_plan.append(actions)
    return parsed_plan


def main():
    env = CoppeliaEnv()
    agent = UnityAgent("GraspAndLift_random_height.onnx", "Place_random_height.onnx")
    seeds = [4]#, 12, 6412, 1233, 2942]
    random.seed(seeds[0])
    np.random.seed(seeds[0])
    init_coppelia_env(env, obstacles=0, object_to_grab=2, n_objects=1, randomize_pos=True)
    state1 = init_planning_env(env.obj_grid_dict, env.half)
    print(OBJ_TO_HANDLE)
    task_planning = plan(state1, [("get_side", "bottle1"), ("leave", "bottle1", "box")], verbose=3)
    print(task_planning)
    parsed_plan = []
    if task_planning:
        parsed_plan = parse_plan(task_planning, env, agent, "bottle1")
        print(parsed_plan)
    else:
        print("No plan found")
    
    print_fancy_grid(state1)

    env.start_simulation()
    for actions in parsed_plan:
        for action, args in actions:
            print(f"Executing: {action.__name__} with args {args}")
            success = action(*args)
            if not success:
                print("Execution failed during:", action.__name__)
                #break
    env.remove_objs()
    env.stop_simulation()


if __name__ == "__main__":
    main()