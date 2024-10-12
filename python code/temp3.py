def normalize_propositions(prop_str):
    """
    Convert the proposition string into a set of atomic propositions.
    Example: 'b & !d & !g' -> {'b', '!d', '!g'}
    """
    return set(prop_str.replace(" ", "").split("&"))

def is_subset(proposition, condition):
    """
    Check if the propositions in 'condition' are a subset of 'proposition'.
    """
    prop_set = normalize_propositions(proposition)
    cond_set = normalize_propositions(condition)
    
    return prop_set <= cond_set

def compare_propositions(prop1, prop2):
    """
    Check if two proposition strings are the same, ignoring the order.
    Example: 'b & !d & !g' and '!g & b & !d' are considered equal.
    """
    prop1_set = normalize_propositions(prop1)
    prop2_set = normalize_propositions(prop2)
    
    return prop1_set == prop2_set

def match_buchi_transition_to_label(buchi_transition, labels):
    """
    Compare a Büchi automaton transition with a dictionary of labeled transitions.
    First checks for exact matches, then checks if the Büchi transition
    is a subset of any transition system label.
    """
    for label, label_proposition in labels.items():
        # First check for exact matches
        if compare_propositions(buchi_transition, label_proposition):
            print(f"Exact match found for {label}: {label_proposition}")
            return label
        
        # If no exact match, check for subset match
        if is_subset(buchi_transition, label_proposition):
            print(f"Subset match found for {label}: {label_proposition}")
            return label
    
    # If no match is found
    print(f"No match found for Büchi transition: {buchi_transition}")
    return None

labels = {
    'c_11': 'b & !d & !g',
    'c_13': 'd & !b & !g',
    'c_23': '!b & !d & !g',
    'c_32': 'g & !b & !d'
}

# Example Büchi transition string
buchi_trans = '!d & !g'

# Match Büchi transition to the transition system labels
matched_label = match_buchi_transition_to_label(buchi_trans, labels)
print(f"Matched label: {matched_label}")

# Example usage:

# # Buchi transition string
# buchi_trans = '!b & !d & !g'
# # Finite transition system string
# finite_trans = '!d & !g'

# # Check if finite_trans is a subset of buchi_trans
# print(is_subset(buchi_trans, finite_trans))  # Output: True

# # Compare jumbled strings
# trans1 = 'b & !d & !g'
# trans2 = '!g & b & !d'
# print(compare_propositions(trans1, trans2))  # Output: True

# s1 = normalize_propositions(buchi_trans)
# print(s1)








def find_accepting_run(product_aut, init_states, accept_states):
    """
    Finds an accepting run with prefix + suffix^w structure.
    
    Args:
    - product_aut: The product automaton (as a graph)
    - init_states: A list of initial states in the product automaton
    - accept_states: A list of accepting states in the product automaton
    
    Returns:
    - A tuple (prefix, suffix) where:
        - prefix: A list of states from the initial state to an accepting state
        - suffix: A list of states representing the cycle (repeated indefinitely)
    """
    start = init_states[0]  # Assuming there's only one initial state
    
    # Step 1: Find a prefix path from the initial state to any accepting state
    for accept in accept_states:
        try:
            # Find the shortest path from the initial state to an accepting state
            prefix_path = nx.shortest_path(product_aut, start, accept)
            
            # Step 2: From the accepting state, find a cycle (suffix) that starts and ends at an accepting state
            cycle = None
            for acc_state in accept_states:
                try:
                    # Find a cycle starting and ending at the accepting state
                    cycle = nx.find_cycle(product_aut, acc_state)
                    if cycle:
                        # Extract the cycle states
                        cycle_states = [state for state, _ in cycle]  # Get states from the cycle
                        return prefix_path, cycle_states  # Return prefix and suffix separately
                except (nx.NetworkXNoCycle):
                    continue
        except nx.NetworkXNoPath:
            continue
    
    return None, None  # If no accepting run is found








# Working, but you need the exact LTL formula and initial states

def construct_product_automaton(fts, buchi_aut, labels):
    product = nx.DiGraph()

    # Add product automaton states
    for fts_state in fts.nodes():
        for buchi_state in range(buchi_aut.num_states()):
            product.add_node((fts_state, buchi_state))

    # Define initial states in the product automaton
    initial_states = [(init_state, buchi_aut.get_init_state_number())]

    # Construct the product automaton based on the FTS and Büchi automaton
    for fts_state in fts.nodes():
        if fts_state in labels:
            fts_label = labels[fts_state]
        else:
            fts_label = '!b & !d & !g'
            # fts_label = '!b & !d'
        for buchi_state in range(buchi_aut.num_states()):
            for fts_next in fts.successors(fts_state):
                for buchi_next in buchi_aut.out(buchi_state):
                    buchi_transition = spot.bdd_format_formula(bdd_dict, buchi_next.cond)

                    # Case 1: FTS state is part of the labels dictionary
                    if fts_state in labels:
                        if compare_propositions(buchi_transition, fts_label):
                            # Add edge to the product automaton if they are equal
                            product.add_edge((fts_state, buchi_state), (fts_next, buchi_next.dst))
                    
                    # Case 2: FTS state is not part of the labels dictionary (implied label: '!b & !d & !g')
                    else:
                        if is_subset(buchi_transition, fts_label):
                            # Add edge to the product automaton if the Buchi condition is a subset of the FTS state
                            product.add_edge((fts_state, buchi_state), (fts_next, buchi_next.dst))

    accepting_states = [(fts_state, buchi_state) for fts_state in fts.nodes() 
                        for buchi_state in range(buchi_aut.num_states()) 
                        if buchi_aut.state_is_accepting(buchi_state)]
    return product, initial_states, accepting_states


def find_accepting_run(product_aut, init_states, accept_states):
    """
    Finds an accepting run with prefix + suffix^w structure.
    
    Args:
    - product_aut: The product automaton (as a graph)
    - init_states: A list of initial states in the product automaton
    - accept_states: A list of accepting states in the product automaton
    
    Returns:
    - A tuple (prefix, suffix) where:
        - prefix: A list of states from the initial state to an accepting state
        - suffix: A list of states representing the cycle (repeated indefinitely)
    """
    start = init_states[0]  # Assuming there's only one initial state
    
    for accept in accept_states:
        try:
            # Find the shortest path from the initial state to an accepting state
            prefix_path = nx.shortest_path(product_aut, start, accept)
            
            # Find a cycle that includes the accepting state
            cycle = None
            successors = list(product_aut.successors(accept))
            for succ in successors:
                if succ == accept:
                    continue  # Skip self-loops for now
                try:
                    # Find a path from the successor back to the accepting state
                    cycle_path = nx.shortest_path(product_aut, succ, accept)
                    if cycle_path:
                        # Ensure the cycle starts with a successor and ends with the accepting state
                        cycle = cycle_path
                        return prefix_path, cycle
                except nx.NetworkXNoPath:
                    continue
            
            # If no cycle found through successors, check for a self-loop
            if product_aut.has_edge(accept, accept):
                # For a self-loop, we need to find a different successor to start the cycle
                for alt_succ in successors:
                    if alt_succ != accept:
                        return prefix_path, [alt_succ, accept]
                # If no other successor found, we have to use the self-loop
                return prefix_path, [accept]
            
        except nx.NetworkXNoPath:
            continue
    
    return None, None  # If no accepting run is found


if accepting_run:
    prefix, suffix = accepting_run  # Unpack the returned tuple
    # Project the prefix and suffix onto the original FTS
    fts_prefix = [state[0] for state in prefix]
    fts_suffix = [state[0] for state in suffix]
    
    print("Found satisfying run in FTS:")
    print("Prefix:", fts_prefix)
    print("Suffix:", fts_suffix)

    # Buchi sequence
    fts_prefix = [state[1] for state in prefix]
    fts_suffix = [state[1] for state in suffix]
    
    print("Found satisfying run in Buchi:")
    print("Prefix:", fts_prefix)
    print("Suffix:", fts_suffix)

else:
    print("No satisfying run found")

