import pyhop
from pyhop_domain import print_fancy_grid

# Initial state
state = pyhop.State("state1")

state.locations = ["box", "waste", "A_box", "home", "left_side", "right_side"]
state.box_on_the_left = False

state.rows, state.cols = (5, 5)
state.grid = [["free" for _ in range(state.cols)] for _ in range(state.rows)]

state.near = {
    "box": "A_box",
    "A_box": "box",
}

state.objects = [
    "cylinder1", "cylinder2", "cylinder3",
    "cylinder4", "cylinder5", "cylinder6",
    "bottle1", "bottle2", "glass1", "pillar",
]
state.movable = [
    "cylinder1", "cylinder2", "cylinder3",
    "cylinder4", "cylinder5", "cylinder6",
    "bottle1", "bottle2",
]

state.robot_at = "home"
state.holding  = None
state.box      = None

# Object placement (mirrors planning_test_3.py)
state.grid[2][1] = "cylinder1"
state.grid[1][3] = "bottle1"
state.grid[3][0] = "cylinder4"
state.grid[4][4] = "cylinder6"


# Planning
print("Initial grid:")
print_fancy_grid(state)

# Grasp bottle1 from the front, then place it in the box
plan = pyhop.pyhop(state, [("get_front", "bottle1"), ("leave", "bottle1", "box")], verbose=3)

print("\nPlan:", plan)
print("\nFinal grid:")
print_fancy_grid(state)
