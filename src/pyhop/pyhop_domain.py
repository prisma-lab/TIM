from __future__ import print_function
from copy import deepcopy
import sys
import pyhop

## NOTE: Pyhop from:
#   https://bitbucket.org/dananau/pyhop/src/master/

# Initializing the PyHOP domain

'''
EXAMPLE:

  0 1 2
0 # # #
1 # O #
2 # # #

    R
'''

OBJECT_TO_REMOVE = "bottle1"

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

#-- helping functions 
def at(state, obj):
    """ Check if a location of an object """
    for i in range(state.rows):
        for j in range(state.cols):
            if state.grid[i][j] == obj:
                return i,j
    return None

def near(state, loc):
    """ Get location that is "near" loc """
    if isinstance(loc, str):
        return state.near[loc]
    return None

def left_obs(state, obj):
    """Return the first obstacle blocking obj from the left (column-decreasing direction)."""
    obs = []
    pos = at(state,obj)
    if pos == None:
        return []
    for i in range(state.rows):
        for j in range(pos[1]):
            if state.grid[i][j] != "free" and state.grid[i][j] != obj and state.grid[i][j] != OBJECT_TO_REMOVE:
                obs.append(state.grid[i][j])  # left-ordered
                return obs
    return obs

def right_obs(state, obj):
    """Return the first obstacle blocking obj from the right (column-increasing direction)."""
    pos = at(state,obj)
    obs = []
    if pos == None:
        return []
    for i in range(state.rows):
        for j in range(pos[1]+1,state.cols):
            if state.grid[i][j] != "free" and state.grid[i][j] != obj and state.grid[i][j] != OBJECT_TO_REMOVE:
                obs.append(state.grid[i][j])  # right-ordered
                return obs
    return obs

def front_obs(state, obj):
    """Return the first obstacle blocking obj from the front (row-increasing direction)."""
    pos = at(state,obj)
    obs = []
    if pos == None:
        return []
    for i in range(pos[0]+1,state.rows):
        for j in range(state.cols):
            if state.grid[i][j] != "free" and state.grid[i][j] != obj and state.grid[i][j] != OBJECT_TO_REMOVE:
                obs.append(state.grid[i][j])  # front-ordered
                return obs
    return obs


def left_free(state, obj):
    """ Check if object is free on the left """
    if left_obs(state,obj) == []:
        return True
    return False

def right_free(state, obj):
    """ Check if object is free on the right """
    if right_obs(state,obj) == []:
        return True
    return False

def front_free(state, obj):
    """ Check if object is free on the front """
    if front_obs(state,obj) == []:
        return True
    return False

def reachable(state, obj):
    """ Check if object is free on some side """
    if front_free(state,obj) or left_free(state,obj) or right_free(state,obj):
        return True
    return False

#--


# Primitive Actions

#  NOTE: this is a learned primitive implmenting a grasping+lifting operation
def pickUp(state, mode, obj):
    """ Pick up an object if reachable. """
    #if (mode == "left" and not left_free(state,obj)) or (mode == "right" and not right_free(state,obj)) or (mode == "up" and not front_free(state,obj)):
    #    return False
    pos = at(state,obj)
    if state.holding == None:
        state.holding = obj
        state.grid[pos[0]][pos[1]] = "free"
        state.robot_at = pos
        return state
    return False

# NOTE: this is a learned primitive implementing an approaching operation toward the releasing pose
def place(state, obj, loc):
    """ Move the robot if the location is reachable. """
    # The object must be grasped and the robot must be at or near the releasing location
    if state.holding == obj and (state.robot_at == loc or near(state, state.robot_at) == loc):
        state.robot_at = loc
        return state
    return False

def move(state, from_loc, to_loc):
    """ Move the robot if the location is reachable. """
    # NOTE: we assume all locations reachable for now?
    #if reachable(state, from_loc, to_loc, "move"):
    state.robot_at = to_loc
    return state

def release(state, obj, loc):
    """ Release an object if it is being held. """
    if state.holding == obj:
        state.holding = None
        if loc in state.locations:
            if loc == "box":
                state.box = obj
            # else, object just disappare
        else:
            state.grid[loc[0]][loc[1]] = obj
        return state
    return False

# High-Level Methods
'''
def m_take_left(state, obj):
    """ HTN method for picking up an object. """
    pos = at(state,obj)
    if pos != None and state.holding == None and obj in state.movable and left_free(state,obj):
        return [("move", state.robot_at, (pos[0]-1, pos[1])), ("pickUp", "side", obj), ("move", state.robot_at, "home")]
    return False

def m_take_right(state, obj):
    """ HTN method for picking up an object. """
    pos = at(state,obj)
    if pos != None and state.holding == None and obj in state.movable and right_free(state,obj):
        return [("move", state.robot_at, (pos[0]+1, pos[1])), ("pickUp", "side", obj), ("move", state.robot_at, "home")]
    return False
'''

def m_take_left(state, obj):
    """ HTN method for picking up an object. """
    pos = at(state,obj)
    if pos != None and state.holding == None and obj in state.movable and left_free(state,obj) and not state.box_on_the_left:
        return [("move", state.robot_at, "left_side"), ("pickUp", "side", obj), ("move", at(state,obj), "home")]
    return False

def m_take_right(state, obj):
    """ HTN method for picking up an object. """
    pos = at(state,obj)
    if pos != None and state.holding == None and obj in state.movable and right_free(state,obj) and state.box_on_the_left:
        return [("move", state.robot_at, "right_side"), ("pickUp", "side", obj), ("move", at(state,obj), "home")]
    return False

def m_take_front(state, obj):
    """ HTN method for picking up an object. """
    pos = at(state,obj)
    if pos != None and state.holding == None and obj in state.movable and front_free(state,obj):
        return [("move", state.robot_at, at(state,obj)), ("pickUp", "up", obj), ("move", at(state,obj), "home")]
    return False

# put an object gently somewhere
def m_put(state, obj, loc):
    """ HTN method for releasing an object. """
    
    # NOTE: the "holding" is supreflous as it is done also on the operator
    if state.holding == obj:
            return [("move", state.robot_at, loc), ("release", obj, loc)]
    return False

# leave an object somewhere, disregarding its final pose
def m_leave(state, obj, loc):
    """ HTN method for releasing an object. """
    
    near_loc = near(state, loc)

    # NOTE: the "holding" is supreflous as it is done also on the operator
    if state.holding == obj and near_loc != None:
            return [("move", state.robot_at, near_loc), ("place", obj, loc), ("release", obj, loc), ("move", near_loc, "home")]
    return False

def m_remove(state, obj):
    """ HTN method for releasing an object. """
    # NOTE: this if is supreflous as it is done also on the operator
    if state.holding == None:
        #return [("take", obj), ("leave", obj, "waste")]
        return [("get", obj), ("put", obj, "waste")]
    return False

def m_clear_left(state, obj):
    """Remove all left-side obstacles blocking obj (only when the box is on the right)."""
    if state.box_on_the_left:
        return False
    obs = left_obs(state,obj)
    tasks = []
    if obj == "pillar":
        print("Trying to clear a pillar...")
    for o in obs:
        if o not in state.movable:
            return False
        tasks.append(("remove", o))
    return tasks

def m_clear_right(state, obj):
    """Remove all right-side obstacles blocking obj (only when the box is on the left)."""
    if not state.box_on_the_left:
        return False
    obs = right_obs(state,obj)
    tasks = []
    if obj == "pillar":
        print("Trying to clear a pillar...")
    for o in obs:
        if o not in state.movable:
            return False
        tasks.append(("remove", o))
    return tasks

def m_clear_front(state, obj):
    """Remove all front-side obstacles blocking obj."""
    obs = front_obs(state,obj)
    tasks = []
    if obj == "pillar":
        print("Trying to clear a pillar...")
    for o in obs:
        if o not in state.movable:
            return False
        tasks.append(("remove", o))
    return tasks

# def m_clear(state, obj, side):
#     if side == "side":
#         if state.box_on_the_left:
#             return m_clear_right(state, obj)
#         else:
#             return m_clear_left(state, obj)
#     elif side == "up":
#         return m_clear_front(state, obj)
#     return False

def m_get(state, obj):
    """Pick up obj from any free side; clear blockers first if needed."""
    if obj not in state.movable:
        return False
    if reachable(state, obj):
        return [("take", obj)]
    return [("clear", obj), ("get", obj)]

def m_get_side(state, obj):
    """Pick up obj from the side opposite to the box; clear side blockers first if needed."""
    if obj not in state.movable:
        return False
    if state.box_on_the_left and right_free(state, obj):
        return [("take_side", obj)]
    if not state.box_on_the_left and left_free(state, obj):
        return [("take_side", obj)]
    return [("clear_side", obj), ("get_side", obj)]


def m_get_front(state, obj):
    """Pick up obj from the front; clear front blockers first if needed."""
    if obj not in state.movable:
        return False
    if front_free(state, obj):
        return [("take_front", obj)]
    return [("clear_front", obj), ("get_front", obj)]


# Registering operators and methods in PyHOP
pyhop.declare_operators(pickUp, place, release, move)
pyhop.declare_methods("take", m_take_front, m_take_left, m_take_right)
pyhop.declare_methods("take_side", m_take_left, m_take_right)
pyhop.declare_methods("take_front", m_take_front)
#pyhop.declare_methods("take", m_take)
pyhop.declare_methods("leave", m_leave)
pyhop.declare_methods("remove", m_remove)
pyhop.declare_methods("clear", m_clear_front, m_clear_left, m_clear_right)
pyhop.declare_methods("clear_side", m_clear_left, m_clear_right)
pyhop.declare_methods("clear_front", m_clear_front)
#pyhop.declare_methods("clear", m_clear)
pyhop.declare_methods("put", m_put)
pyhop.declare_methods("get", m_get)
pyhop.declare_methods("get_side", m_get_side)
pyhop.declare_methods("get_front", m_get_front)