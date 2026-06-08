import random

def extract_actions(domain_str):
    actions = []
    lines = domain_str.splitlines()
    i = 0

    while i < len(lines):
        line = lines[i].strip()
        if line.startswith("(:action"):
            action_block = []
            paren_count = 0
            in_action = False

            while i < len(lines):
                current_line = lines[i]
                action_block.append(current_line)

                paren_count += current_line.count("(")
                paren_count -= current_line.count(")")

                if not in_action:
              
                    tokens = current_line.replace("(", "").split()
                    if len(tokens) >= 2:
                        action_name = tokens[1]
                        in_action = True

                if in_action and paren_count == 0:
                    break  
                i += 1

            full_block = "\n".join(action_block)
            actions.append((action_name, full_block))
        else:
            i += 1

    return actions


def extract_probabilistic_block(full_action_block):
    start_idx = full_action_block.find(":effect")
    if start_idx == -1:
        return None, -1, -1

    effect_start = full_action_block[start_idx:]
    prob_start_idx = effect_start.find("(probabilistic")
    if prob_start_idx == -1:
        return None, -1, -1

    abs_prob_start = start_idx + prob_start_idx
    paren_count = 0
    end_idx = None

    for i, char in enumerate(full_action_block[abs_prob_start:]):
        if char == '(':
            paren_count += 1
        elif char == ')':
            paren_count -= 1
            if paren_count == 0:
                end_idx = abs_prob_start + i
                break

    if end_idx is not None:
        return full_action_block[abs_prob_start:end_idx + 1], abs_prob_start, end_idx + 1
    else:
        return None, -1, -1

def sample_effect_from_prob_block(prob_block: str, seed=None):
    if seed is not None:
        random.seed(seed)


    prob_block = prob_block.strip()
    if not prob_block.startswith("(probabilistic"):
        raise ValueError("Block must start with '(probabilistic'")

    content = prob_block[len("(probabilistic"):].strip()
    if content and content[0] == '\n':
        content = content[1:]

    i = 0
    length = len(content)
    effects = []

    while i < length:
        while i < length and content[i].isspace():
            i += 1
        if i >= length:
            break
        prob_start = i
        while i < length and (content[i].isdigit() or content[i] in ".eE-+"):
            i += 1
        prob_str = content[prob_start:i].strip()
        if not prob_str:
            break
        prob = float(prob_str)

        while i < length and content[i].isspace():
            i += 1

        if i >= length or content[i] != '(':
            raise ValueError(f"Expected '(' after probability {prob}, got: {content[i:i+10]}")
        paren_start = i
        paren_count = 0
        while i < length:
            if content[i] == '(':
                paren_count += 1
            elif content[i] == ')':
                paren_count -= 1
                if paren_count == 0:
                    i += 1 
                    break
            i += 1
        effect_str = content[paren_start:i].strip()
        effects.append((prob, effect_str))

    if not effects:
        raise ValueError("No valid effects found inside probabilistic block.")

   
    probs, effect_bodies = zip(*effects)
    chosen_idx = random.choices(range(len(probs)), weights=probs)[0]
    return effect_bodies[chosen_idx]


def replace_probabilistic_with_sampled_effect(full_action_block, seed=None):

    prob_block, start_idx, end_idx = extract_probabilistic_block(full_action_block)
    
    if prob_block is None:
        return full_action_block  

    sampled_effect = sample_effect_from_prob_block(prob_block, seed=seed)
    if sampled_effect is None:
        return full_action_block

    new_action_block = (
        full_action_block[:start_idx] +
        sampled_effect +
        full_action_block[end_idx:]
    )
    return new_action_block

def determinize_domain(action_blocks, domain_str, seed=None):
    new_domain = domain_str
    for action_name, full_block in action_blocks:
        if ":effect" in full_block and "probabilistic" in full_block:
            new_block = replace_probabilistic_with_sampled_effect(full_block, seed=seed)
            new_domain = new_domain.replace(full_block, new_block)

    return new_domain
