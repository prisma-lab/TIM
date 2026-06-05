import eventlet
eventlet.monkey_patch()

from flask import Flask, render_template
from flask_socketio import SocketIO
import random
import numpy as np

# Importa le tue classi
from planning_and_exec import init_coppelia_env, init_planning_env, plan, parse_plan, CoppeliaEnv, UnityAgent

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, async_mode='eventlet', cors_allowed_origins="*")

stop_thread = False

@app.route('/')
def index():
    return render_template('index.html')

def get_grid_data(state):
    rows = state.rows
    cols = state.cols
    grid_data = []

    for r in range(rows):
        row_data = []
        for c in range(cols):
            obj = state.grid[r][c]
            cell_type = "empty"
            
            if obj == "free": cell_type = "empty"
            elif obj.startswith("cylinder"): cell_type = "cylinder"
            elif obj.startswith("bottle"): cell_type = "bottle"
            elif obj.startswith("glass"): cell_type = "glass"
            elif obj == "pillar": cell_type = "pillar"
            else: cell_type = "obstacle"

            # Controllo Coordinate Robot
            is_robot_here = False
            if isinstance(state.robot_at, (list, tuple, np.ndarray)):
                if state.robot_at[0] == r and state.robot_at[1] == c:
                    is_robot_here = True

            row_data.append({'type': cell_type, 'has_robot': is_robot_here})
        grid_data.append(row_data)
    
    # Controllo Etichette Esterne (Stringhe)
    robot_label = state.robot_at if isinstance(state.robot_at, str) else None
    
    return {
        'grid': grid_data, 
        'robot_label': robot_label,
        'holding': state.holding
    }

def find_object_coords(state, obj_name):
    """Cerca un oggetto nella griglia e restituisce [r, c] o None"""
    for r in range(state.rows):
        for c in range(state.cols):
            if state.grid[r][c] == obj_name:
                return [r, c]
    return None

def execution_logic():
    global stop_thread
    
    socketio.emit('log', {'data': 'Inizializzazione...'})
    socketio.sleep(0.01)
    
    env = CoppeliaEnv()
    agent = UnityAgent("GraspAndLift_random_height.onnx", "Place_random_height.onnx")
    seeds = [7]
    random.seed(seeds[0])
    np.random.seed(seeds[0])
    
    init_coppelia_env(env, obstacles=0, object_to_grab=2, n_objects=4, randomize_pos=True)
    state1 = init_planning_env(env.obj_grid_dict, env.half)
    
    socketio.emit('update_grid', get_grid_data(state1))
    
    tasks = [("get_side", "bottle1"), ("leave", "bottle1", "box")]
    task_planning = plan(state1, tasks, verbose=1)
    
    if not task_planning: return

    parsed_plan = parse_plan(task_planning, env, agent, "bottle1")
    
    # Init Grafo
    graph_nodes = []
    graph_edges = []
    for i, action in enumerate(task_planning):
        node_id = f"node_{i}"
        label = f"{action[0]}\n{str(action[1:])}"
        graph_nodes.append({'data': {'id': node_id, 'label': label, 'type': 'plan'}})
        if i > 0:
            prev_id = f"node_{i-1}"
            graph_edges.append({'data': {'source': prev_id, 'target': node_id}})     
    socketio.emit('init_graph', {'nodes': graph_nodes, 'edges': graph_edges})
    socketio.sleep(1)

    env.start_simulation()
    execution_failed = False

    for i, sub_actions in enumerate(parsed_plan):
        if stop_thread or execution_failed: break
        
        hl_node_id = f"node_{i}"
        socketio.emit('highlight_node', {'id': hl_node_id, 'status': 'running'})
        
        current_hl_action = task_planning[i]
        hl_action_type = current_hl_action[0]

        for j, (action_func, args) in enumerate(sub_actions):
            if stop_thread: break

            func_name = action_func.__name__
            ll_node_id = f"method_{i}_{j}" 

            socketio.emit('add_method_node', {
                'parent_id': hl_node_id, 
                'id': ll_node_id, 
                'label': f"{j+1}. {func_name}",
                'args': "...",
                'sub_index': j 
            })
            socketio.sleep(0.001)
            
            try:
                success = action_func(*args)
            except Exception as e:
                print(f"Errore Exception: {e}")
                success = False

            status = 'success' if success else 'failure'
            socketio.emit('update_method_node', {'id': ll_node_id, 'status': status})
            
            if not success:
                socketio.emit('highlight_node', {'id': hl_node_id, 'status': 'failure'})
                execution_failed = True
                break
            
            if j == len(sub_actions) - 1:
                if hl_action_type == "move":
                    destination = current_hl_action[2]
                    state1.robot_at = destination

                elif hl_action_type == "pickUp":
                    target_obj = current_hl_action[2]
                    coords = find_object_coords(state1, target_obj)
                    if coords:
                        state1.robot_at = coords
                        state1.grid[coords[0]][coords[1]] = "free"
                        state1.holding = target_obj

                elif hl_action_type == "release" or hl_action_type == "place":
                    state1.holding = None

                socketio.emit('update_grid', get_grid_data(state1))

        if not execution_failed:
            socketio.emit('highlight_node', {'id': hl_node_id, 'status': 'success'})

    env.remove_objs()
    env.stop_simulation()
    socketio.emit('log', {'data': 'Simulazione terminata.'})

@socketio.on('start_simulation')
def handle_start_sim():
    global stop_thread
    stop_thread = False
    socketio.start_background_task(target=execution_logic)

@socketio.on('stop_simulation')
def handle_stop_sim():
    global stop_thread
    stop_thread = True

if __name__ == '__main__':
    socketio.run(app, debug=True, port=5000)