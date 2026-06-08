import time
import os
import argparse
import torch
import numpy as np
import environment


buffer = []

def collect_rollout(env):
    action = env.full_random_action()
    from_obj = action[0]
    to_obj = action[1]
    cg = env.get_contact_graph()
    from_contacts, = np.where(cg[from_obj] > 0)
    to_contacts, = np.where(cg[to_obj] > 0)
    valid_objs = np.concatenate(([from_obj], [to_obj], from_contacts, to_contacts), axis=0)
    valid_objs = np.unique(valid_objs)
    pre_position, effect = env.step(*action)
    action[0] = np.where(valid_objs == action[0])[0][0]
    action[1] = np.where(valid_objs == action[1])[0][0]

    post_position = env.state()
    pre_position = pre_position[valid_objs]
    post_position = post_position[valid_objs]
    effect = effect[valid_objs]
    return pre_position, action, effect, post_position


if __name__ == "__main__":
    parser = argparse.ArgumentParser("Explore environment.")
    parser.add_argument("-N", help="number of interactions", type=int, required=True)
    parser.add_argument("-T", help="interaction per episode", type=int, required=True)
    parser.add_argument("-o", help="output folder", type=str, required=True)
    parser.add_argument("-i", help="offset index", type=int, required=True)
    parser.add_argument("-n_min", help="minimum number of objects", type=int, default=2)
    parser.add_argument("-n_max", help="maximum number of objects", type=int, default=4)
    args = parser.parse_args()

    if not os.path.exists(args.o):
        os.makedirs(args.o)

    states = torch.zeros(args.N, args.n_max, 11, dtype=torch.float)
    actions = torch.zeros(args.N, 8, dtype=torch.int)
    effects = torch.zeros(args.N, args.n_max, 14, dtype=torch.float)
    post_states = torch.zeros(args.N, args.n_max, 11, dtype=torch.float)
    masks = torch.zeros(args.N, dtype=torch.int)
    env = environment.BlocksWorld_v4(gui=0, min_objects=args.n_min, max_objects=args.n_max)
    np.random.seed()

    start = time.time()
    env_it = 0
    i = 0

    while i < args.N:
        if (env_it) == args.T:
            env_it = 0
            env.reset_objects()

        pre_position, action, effect, post_position = collect_rollout(env)
        n_objs = pre_position.shape[0]
        env_it += 1
        states[i, :n_objs] = torch.tensor(pre_position, dtype=torch.float)
        actions[i] = torch.tensor(action, dtype=torch.int)
        masks[i] = n_objs
        effects[i, :n_objs] = torch.tensor(effect, dtype=torch.float)
        post_states[i, :n_objs] = torch.tensor(post_position, dtype=torch.float)

        i += 1
        if i % (args.N // 100) == 0:
            print(f"Proc {args.i}: {100*i/args.N}% completed.")

    torch.save(states, os.path.join(args.o, f"state_{args.i}.pt"))
    torch.save(actions, os.path.join(args.o, f"action_{args.i}.pt"))
    torch.save(masks, os.path.join(args.o, f"mask_{args.i}.pt"))
    torch.save(effects, os.path.join(args.o, f"effect_{args.i}.pt"))
    torch.save(post_states, os.path.join(args.o, f"post_state_{args.i}.pt"))
    end = time.time()
    del env
    print(f"Completed in {end-start:.2f} seconds.")
