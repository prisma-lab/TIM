import argparse
import os
import pickle
from itertools import permutations
from copy import deepcopy
import torch
import numpy as np
from tqdm import tqdm

import dataset
from models import load_ckpt
from utils import mask_unknown


class State:
    def __init__(self, obj_tensor={}, rel_tensor=[], act_tensor=None, act_symbol=None):
        assert act_tensor is not None or act_symbol is not None
        self.n_obj = obj_tensor.shape[0]
        self.n_rel = rel_tensor.shape[0]
        obj_dict = {}
        relations = [{} for _ in range(self.n_rel)]
        for i in range(self.n_obj):
            obj_dict[i] = tuple(obj_tensor[i].int().tolist())
            for j in range(self.n_obj):
                for k in range(self.n_rel):
                    relations[k][(i, j)] = int(rel_tensor[k, i, j])
        if act_symbol is not None:
            action = act_symbol
        else:
            action = tensor_to_action(act_tensor)
        self.obj_dict = obj_dict
        self.relations = relations
        self.action = action

    def substitute(self, delta):
        obj_dict = {}
        relations = [{} for _ in range(self.n_rel)]
        for idx in delta:
            key = delta[idx]
            obj_dict[key] = self.obj_dict[idx]
            for idx2 in delta:
                key2 = delta[idx2]
                for k in range(self.n_rel):
                    relations[k][(key, key2)] = self.relations[k][idx, idx2]
        if len(self.action) == 4:
            action = (delta[self.action[0]], self.action[1], delta[self.action[2]], self.action[3])
        else: 
            action = self.action
        new_state = deepcopy(self)
        new_state.obj_dict = obj_dict
        new_state.relations = relations
        new_state.action = action
        return new_state

    def get_params(self):
        params = []
        for key in self.obj_dict:
            params.append(key)
        for k, rel_dict in enumerate(self.relations):
            for (key1, key2) in rel_dict:
                params.append(key1)
                params.append(key2)
        return tuple(np.unique(params).tolist())

    def __repr__(self):
        return f"Obj: {self.obj_dict}\n" + \
               f"Relations: {self.relations}\n" + \
                f"Action: {self.action}"

    def __eq__(self, other):
        return self.obj_dict == other.obj_dict and \
               self.relations == other.relations and \
               self.action == other.action

    def __hash__(self):
        repr = str(sorted(tuple(self.obj_dict.items()))) + \
               str(tuple(sorted(tuple(k.items())) for k in self.relations)) + \
               str(self.action)
        return hash(repr)


class Effect:
    def __init__(self, z_i, r_i, z_f, r_f):
        obj_diff_idx = torch.where(z_i != z_f)[0].unique()
        obj_dict = {}
        for idx in obj_diff_idx:
            obj_dict[idx.item()] = tuple((z_f[idx].int() - z_i[idx].int()).tolist())
        self.z_eff = obj_dict

        rel_diffs = torch.where(r_i != r_f)
        relations = [{} for _ in range(r_i.shape[0])]
        for k, i, j in zip(*rel_diffs):
            relations[k][(int(i), int(j))] = int(r_f[k, i, j]) - int(r_i[k, i, j])
        self.r_eff = relations

    def __eq__(self, other):
        return self.z_eff == other.z_eff and \
               self.r_eff == other.r_eff

    def substitute(self, delta):
        z_eff = {}
        for k, v in self.z_eff.items():
            z_eff[delta[k]] = v
        r_eff = []
        for rel_dict in self.r_eff:
            new_rel_dict = {}
            for k, v in rel_dict.items():
                key = tuple(delta[k_i] for k_i in k)
                new_rel_dict[key] = v
            r_eff.append(new_rel_dict)
        new_effect = deepcopy(self)
        new_effect.z_eff = z_eff
        new_effect.r_eff = r_eff
        return new_effect

    def __repr__(self):
        return f"Obj Eff: {self.z_eff}\n" + \
               f"Rel Eff: {self.r_eff}"

    def __hash__(self):
        repr = str(sorted(tuple(self.z_eff.items()))) + \
               str(tuple(sorted(tuple(k.items())) for k in self.r_eff))
        return hash(repr)


def compute_operators(loader):
    preconditions = {}
    operators = {}
    for t, sample in enumerate(loader):
        z_i, r_i, a, z_f, r_f = preprocess(*sample)
        if type(a) == tuple:  
            z_gr = State(z_i, r_i, act_symbol=a)
        else:
            z_gr = State(z_i, r_i, a)
        indices = z_gr.get_params()
        names = [f"o{i}" for i in range(len(indices))]
        subs = []
        for perm in permutations(indices):
            subs.append({perm[i]: names[i] for i in range(len(perm))})

        found_match = False
        for sub in subs:
            z_abs = z_gr.substitute(sub)
            if z_abs in preconditions:
                preconditions[z_abs].append((t, sub))
                found_match = True
                break
        if not found_match:
            preconditions[z_gr.substitute(subs[0])] = [(t, subs[0])]


    sorted_pre = sorted(preconditions.items(), key=lambda x: len(x[1]), reverse=True)
    for i, (precond, transitions) in enumerate(sorted_pre):
        effects = {}
        for (idx, sub) in transitions:
            z_i, r_i, _, z_f, r_f, _ = trainset[idx]
            eff_abs = Effect(z_i, r_i, z_f, r_f).substitute(sub)
            if eff_abs in effects:
                effects[eff_abs] += 1
            else:
                effects[eff_abs] = 1
        operators[precond] = effects

    return operators


def preprocess(z_i, r_i, a, z_f, r_f, m):
    if z_i.shape[1] == 1:  
        z_i = z_i[0]
        r_i = r_i[0]
        z_f = z_f[0]
        r_f = r_f[0]
        a = a[0]
        from_obj_idx = torch.where(a[:, 0] > 0.5)[0][0].cpu().numpy()
        to_obj_idx = torch.where(a[:, 4] > 0.5)[0][0].cpu().numpy()
        if from_obj_idx == to_obj_idx:
            a = (0, int(a[from_obj_idx, 2]), int(a[to_obj_idx, 6]))
        else:
            a = (1, int(a[from_obj_idx, 2]), int(a[to_obj_idx, 6]))
        return z_i, r_i, a, z_f, r_f
    z_i = z_i[0, m[0]]
    z_f = z_f[0, m[0]]
    a = a[0, m[0]]
    n_obj = z_i.shape[0]
    mm = (m.T.float() @ m.float()).bool()
    r_i = r_i[0, :, mm].reshape(-1, n_obj, n_obj)
    r_f = r_f[0, :, mm].reshape(-1, n_obj, n_obj)
    return z_i, r_i, a, z_f, r_f


def tensor_to_action(action):
    from_action_idx = int(torch.where(action[:, 0] == 1)[0][0])
    from_position = int(action[from_action_idx, 2])
    to_action_idx = int(torch.where(action[:, 4] == 1)[0][0])
    to_position = int(action[to_action_idx, 6])
    action_tup = (from_action_idx, from_position, to_action_idx, to_position)
    return action_tup


def construct_domain(action_schemas, latent_dim, relation_dim, probabilistic=False):
    domain = "(define (domain blocks)\n"
    domain += "\t(:requirements :equality"
    if probabilistic:
        domain += " :probabilistic-effects"
    domain += ")\n"
    domain += "\t(:predicates\n"
    for i in range(latent_dim):
        domain += f"\t\t(z{i} ?x)\n"
        domain += f"\t\t(not_z{i} ?x)\n"
    for i in range(relation_dim):
        domain += f"\t\t(r{i} ?x ?y)\n"
        domain += f"\t\t(not_r{i} ?x ?y)\n"
    domain += "\t)\n"
    for schema in action_schemas:
        domain += schema
    domain += ")"
    return domain


def precond_to_pddl(params, z_i, r_i, indentation="\t\t"):
    schema = f"(and\n{indentation}"
    for i, p1 in enumerate(params):
        for p2 in params[i+1:]:
            schema += f"(not (= {p1} {p2})) "
    schema += "\n"
    for name, obj_val in z_i.items():
        if len(obj_val) != 0:
            schema += indentation
        for j, val in enumerate(obj_val):
            if val == 1:
                schema += f"(z{j} ?{name}) "
            elif val == 0:
                schema += f"(not_z{j} ?{name}) "
        if len(obj_val) != 0:
            schema += "\n"
    for k, rel_dict in enumerate(r_i):
        if len(rel_dict) != 0:
            schema += indentation
        for (name1, name2), val in rel_dict.items():
            if val == 1:
                schema += f"(r{k} ?{name1} ?{name2}) "
            elif val == 0:
                schema += f"(not_r{k} ?{name1} ?{name2}) "
        if len(rel_dict) != 0:
            schema += "\n"
    schema += "\t)\n"
    return schema


def single_effect_to_pddl(precond, z_eff, r_eff, indentation="\t\t"):
    schema_parts = []
    changed_z = {} 
    changed_r = {} 
    for name, obj_val in z_eff.items():
        for j, val in enumerate(obj_val):
            changed_z[(name, j)] = True
            if val == 1:
                schema_parts.append(f"(z{j} ?{name}) (not (not_z{j} ?{name}))")
            elif val == -1:
                schema_parts.append(f"(not_z{j} ?{name}) (not (z{j} ?{name}))")
            elif val == 3:
                schema_parts.append(f"(not (not_z{j} ?{name}))")
            elif val == -3:
                schema_parts.append(f"(not_z{j} ?{name})")
            elif val == 2:
                schema_parts.append(f"(not (z{j} ?{name}))")
            elif val == -2:
                schema_parts.append(f"(z{j} ?{name})")
        if len(obj_val) != 0:
            schema_parts.append("\n")
    for k, rel_tuple in enumerate(r_eff):
        for (name1, name2), val in rel_tuple.items():
            changed_r[(k, name1, name2)] = True
            if val == 1:
                schema_parts.append(f"(r{k} ?{name1} ?{name2}) (not (not_r{k} ?{name1} ?{name2}))")
            elif val == -1:
                schema_parts.append(f"(not_r{k} ?{name1} ?{name2}) (not (r{k} ?{name1} ?{name2}))")
            elif val == 3:
                schema_parts.append(f"(not (not_r{k} ?{name1} ?{name2}))")
            elif val == -3:
                schema_parts.append(f"(not_r{k} ?{name1} ?{name2})")
            elif val == 2:
                schema_parts.append(f"(not (r{k} ?{name1} ?{name2}))")
            elif val == -2:
                schema_parts.append(f"(r{k} ?{name1} ?{name2})")
        if len(rel_tuple) != 0:
   
            schema_parts.append("\n")

    for name, obj_val in precond.obj_dict.items():
        for j, val in enumerate(obj_val):
            if (name, j) not in changed_z: 
                if val == 1:
                    schema_parts.append(f"(z{j} ?{name})")
                elif val == 0:
                    schema_parts.append(f"(not_z{j} ?{name})")
        schema_parts.append("\n")

    for k, rel_dict in enumerate(precond.relations):
        for (name1, name2), val in rel_dict.items():
            if (k, name1, name2) not in changed_r: 
                if val == 1:
                    schema_parts.append(f"(r{k} ?{name1} ?{name2})")
                elif val == 0:
                    schema_parts.append(f"(not_r{k} ?{name1} ?{name2})")
        schema_parts.append("\n")


    if not schema_parts:
        return "(and)\n"
        
    schema = "(and\n"
    last_part = None
    schema += indentation
    for part in schema_parts:
        if last_part == "\n":
            schema += f"{indentation}{part} "
        else:
            schema += f"{part} "
        last_part = part
    schema += f"\n{indentation[1:]})\n"
    return schema

def effect_to_pddl(precond, effect, probabilistic=False, indentation="\t\t"):
    effects = []
    counts = []
    for eff, c in effect.items():
        effects.append(eff)
        counts.append(c)
    counts = np.array(counts)
    probs = counts / counts.sum()
    prob_int = (probs * 100000).round().astype(int)
    ptotal = prob_int.sum()
    max_idx = np.argmax(prob_int)
    if probabilistic:
        threshold = 0.1
        eff_indices = np.where(probs >= threshold)[0]
        if len(eff_indices) == 0 and len(probs) > 0:
            eff_indices = [np.argmax(probs)]
        
        filtered_effects = [effects[i] for i in eff_indices]
        filtered_probs = probs[eff_indices]
    
        prob_sum = filtered_probs.sum()
        if prob_sum > 0:
            normalized_probs = filtered_probs / prob_sum
        else:
            normalized_probs = filtered_probs

        schema = "(probabilistic\n"

        prob_int = (normalized_probs * 100000).round().astype(int)
        ptotal = prob_int.sum()
        if ptotal != 100000 and len(prob_int) > 0:
            max_idx = np.argmax(prob_int)
            prob_int[max_idx] += 100000 - ptotal
        
    
        for i, eff in enumerate(filtered_effects):
            eff_pddl = single_effect_to_pddl(precond, eff.z_eff, eff.r_eff, indentation=indentation+"\t")
            if prob_int[i] != 100000:
                prob_str = f"0.{prob_int[i]:05d}".rstrip('0')
            else:
                prob_str = "1.0"   
           
            schema += f"{indentation}{prob_str} {eff_pddl}"
        schema += f"{indentation[1:]})\n"
    else:
        max_eff = effects[max_idx]
        max_eff_pddl = single_effect_to_pddl(precond, max_eff.z_eff, max_eff.r_eff, indentation=indentation)
        schema = max_eff_pddl
    return schema

def operator_to_pddl(idx, precond, effect, probabilistic=False):
    z_i = precond.obj_dict
    r_i = precond.relations
    action = precond.action
    params = precond.get_params()

    if len(action) == 4:
        schema = f"(:action {action[0]}_{action[1]}_{action[2]}_{action[3]}_i{idx}"
    else:
        schema = f"(:action {action[0]}_{action[1]}_{action[2]}_i{idx}"
    if probabilistic:
        count = sum(effect.values())
        schema += f"_c{count}\n"
    else:
        max_count = max(effect.values())
        schema += f"_c{max_count}\n"
    params = [f"?{p}" for p in params]
    schema += f"\t:parameters ({' '.join(params)})\n"
    precond_pddl = precond_to_pddl(params, z_i, r_i)
    effect_pddl = effect_to_pddl(precond, effect, probabilistic=probabilistic)
    schema += f"\t:precondition {precond_pddl}"
    schema += f"\t:effect {effect_pddl}"
    schema += ")\n"
    return schema


def tensor_to_pddl_problem(obj_pre, rel_pre, obj_post, rel_post, graph_pre, graph_post):
    problem = "(define (problem dom1)\n"
    problem += "\t(:domain blocks)\n"
    problem += "\t(:objects "
    for i in range(obj_pre.shape[0]):
        problem += f"obj{i} "
    problem += ")\n"
    problem += "\t(:init\n"
    n_obj, z_dim = obj_pre.shape
    n_rel, _, _ = rel_pre.shape
    for i in range(n_obj):
        problem += "\t\t"
        for j in range(z_dim):
            if obj_pre[i, j] == 1:
                problem += f"(z{j} obj{i}) "
            elif obj_pre[i, j] == 0:
                problem += f"(not_z{j} obj{i}) "
        problem += "\n"
    for k in range(n_rel):
        problem += "\t\t"
        for i in range(n_obj):
            for j in range(n_obj):
                if graph_pre[i, j] == 1:
                    if rel_pre[k, i, j] == 1:
                        problem += f"(r{k} obj{i} obj{j}) "
                    elif rel_pre[k, i, j] == 0:
                        problem += f"(not_r{k} obj{i} obj{j}) "
        problem += "\n"
    problem += "\t)\n"
    problem += "\t(:goal (and\n"
    for i in range(n_obj):
        problem += "\t\t"
        for j in range(z_dim):
            if obj_post[i, j] == 1:
                problem += f"(z{j} obj{i}) "
            elif obj_post[i, j] == 0:
                problem += f"(not_z{j} obj{i}) "
        problem += "\n"
    for k in range(n_rel):
        problem += "\t\t"
        for i in range(n_obj):
            for j in range(n_obj):
                if graph_post[i, j] == 1:
                    if rel_post[k, i, j] == 1:
                        problem += f"(r{k} obj{i} obj{j}) "
                    elif rel_post[k, i, j] == 0:
                        problem += f"(not_r{k} obj{i} obj{j}) "
        problem += "\n"
    problem += "\t))\n"
    problem += ")"
    return problem


def collate_preds(model, loader):
    z_i, r_i, a, z_f, r_f, m = [], [], [], [], [], []
    N_EPS = 100
    for _, (state, action, _, mask, next_state) in enumerate(tqdm(loader)):
        state = state.to(model.device)
        next_state = next_state.to(model.device)
        mask = mask.to(model.device)

        state = state.unsqueeze(0).repeat(N_EPS, 1, 1, 1).reshape(-1, *state.shape[1:])
        next_state = next_state.unsqueeze(0).repeat(N_EPS, 1, 1, 1).reshape(-1, *next_state.shape[1:])
        mask = mask.unsqueeze(0).repeat(N_EPS, 1, 1).reshape(-1, *mask.shape[1:])

        z = model.encode(state)
        z = z.reshape(N_EPS, -1, *z.shape[1:])
        zn = model.encode(next_state)
        zn = zn.reshape(N_EPS, -1, *zn.shape[1:])
        z, zn = mask_unknown(z), mask_unknown(zn)

        r = model.attn_weights(state, mask)
        r = r.reshape(N_EPS, -1, *r.shape[1:])
        rn = model.attn_weights(next_state, mask)
        rn = rn.reshape(N_EPS, -1, *rn.shape[1:])
        r, rn = mask_unknown(r), mask_unknown(rn)

        mask = mask.reshape(N_EPS, -1, *mask.shape[1:])
        z[~mask[0].bool()] = 0
        zn[~mask[0].bool()] = 0

        z_i.append(z)
        r_i.append(r)
        a.append(action)
        z_f.append(zn)
        r_f.append(rn)
        m.append(mask[0])
    z_i = torch.cat(z_i).cpu().long()
    r_i = torch.cat(r_i).cpu().long()
    a = torch.cat(a).cpu().long()
    z_f = torch.cat(z_f).cpu().long()
    r_f = torch.cat(r_f).cpu().long()
    m = torch.cat(m).cpu().bool()
    return torch.utils.data.TensorDataset(z_i, r_i, a, z_f, r_f, m)


if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument("-n", type=str, required=True, help="Experiment name")
    args.add_argument("-min_count", type=int, default=100, help="Minimum eff. count required for an operator")
    args = args.parse_args()

    model, _ = load_ckpt(args.n, tag="best")
    model.freeze()
    save_path = os.path.join("save", args.n)
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    dataset_path = os.path.join(save_path, "trainset.pt")
    if not os.path.exists(dataset_path):
        if torch.cuda.is_available():
            model.to("cuda")
        cnt_trainloader = torch.utils.data.DataLoader(
            dataset.StateActionEffectDataset(model.hparams.config["dataset_name"]),
            batch_size=128)
        trainset = collate_preds(model, cnt_trainloader)
        torch.save(trainset, dataset_path)
    else:
        trainset = torch.load(dataset_path, weights_only=False)
    trainloader = torch.utils.data.DataLoader(trainset, batch_size=1)

    operator_path = os.path.join(save_path, "operators.pkl")
    if not os.path.exists(operator_path):
        operators = compute_operators(trainloader)
        pickle.dump(operators, open(operator_path, "wb"))
    else:
        operators = pickle.load(open(operator_path, "rb"))

    prob_action_schemas = []
    det_action_schemas = []

    for i, (precond, effects) in enumerate(operators.items()):       
        max_count = sum(effects.values())
        if max_count > args.min_count:
            prob_action_schemas.append(operator_to_pddl(i, precond, effects, probabilistic=True))
            det_action_schemas.append(operator_to_pddl(i, precond, effects, probabilistic=False))

    prob_domain = construct_domain(prob_action_schemas, model.hparams.config["latent_dim"],
                              model.hparams.config["n_attention_heads"], probabilistic=True)
    
    det_domain = construct_domain(det_action_schemas, model.hparams.config["latent_dim"],
                              model.hparams.config["n_attention_heads"], probabilistic=False)
   
    print(f"Constructed domain with {len(prob_action_schemas)} action schemas.")
    
    with open(os.path.join(save_path, "domain_prob.pddl"), "w") as f:
        f.write(prob_domain)
    with open(os.path.join(save_path, "domain.pddl"), "w") as f:
        f.write(det_domain)
    with open(os.path.join(save_path, "info.txt"), "w") as f:
        f.write(f"Number of operators: {len(operators)}\n")
        f.write(f"Number of actions: {len(det_action_schemas)}\n")
        f.write(f"min_count: {args.min_count} ")