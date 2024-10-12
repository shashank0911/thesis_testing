import networkx as nx
import spot
import pygraphviz as pgv
import pickle
import numpy as np


def normalize_propositions(prop_str):
    """
    Convert the proposition string into a set of atomic propositions.
    Example: 'b & !d & !g' -> {'b', '!d', '!g'}
    """
    return set(prop_str.replace(" ", "").split("&"))

def is_subset(prop1, prop2):
    """
    Check if the propositions in 'condition' are a subset of 'proposition'.
    """
    prop_set_1 = normalize_propositions(prop1)
    prop_set_2 = normalize_propositions(prop2)
    
    return prop_set_1 <= prop_set_2

def compare_propositions(prop1, prop2):
    """
    Check if two proposition strings are the same, ignoring the order.
    Example: 'b & !d & !g' and '!g & b & !d' are considered equal.
    """
    prop_set_1 = normalize_propositions(prop1)
    prop_set_2 = normalize_propositions(prop2)
    
    return prop_set_1 == prop_set_2

def construct_product_automaton(fts, buchi_aut, labels):
    product = nx.DiGraph()
    accepting_states = []

    # Add product automaton states
    for fts_state in fts.nodes():
        for buchi_state in range(buchi_aut.num_states()):
            product.add_node((fts_state, buchi_state))
            if buchi_aut.state_is_accepting(buchi_state):
                accepting_states.append((fts_state, buchi_state))


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

    return product, initial_states, accepting_states


def find_accepting_run(product_aut, init_states, accepting_states):
    """
    Finds an accepting run with prefix + suffix^w structure.
    
    Args:
    - product_aut: The product automaton (as a graph)
    - init_states: A list of initial states in the product automaton
    - accepting_states: A list of accepting states in the product automaton
    
    Returns:
    - A tuple (prefix, suffix) where:
        - prefix: A list of states from the initial state to an accepting state
        - suffix: A list of states representing the cycle (repeated indefinitely)
    """
    start = init_states[0]  # Assuming there's only one initial state
    
    for accept in accepting_states:
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

def save_run_to_file(prefix, suffix, filename='run_result.pkl'):
    """
    Save the prefix and suffix as a single 1D numpy array in a .pkl file.
    
    Args:
    - prefix: List of states in the prefix (can be None)
    - suffix: List of states in the suffix
    - filename: Name of the file to save the data (default: 'run_result.pkl')
    """
    # Combine prefix and suffix
    if prefix is None:
        combined = suffix
    else:
        combined = prefix + suffix
    
    # Convert to 1D numpy array
    array_data = np.array(combined)
    
    # Save to .pkl file
    with open(filename, 'wb') as f:
        pickle.dump(array_data, f)
    
    print(f"Run saved to {filename}")




if __name__ == "__main__":
        
    n_rows = 3
    n_cols = 3

    fts = nx.DiGraph()

    states = [f'c_{i}{j}' for i in range(1, n_rows + 1) for j in range(1, n_cols + 1)]
    fts.add_nodes_from(states)

    for i in range(1, n_rows + 1):
        for j in range(1, n_cols + 1):
            cur_cell = f'c_{i}{j}'
            if i > 1:
                fts.add_edge(cur_cell, f'c_{i-1}{j}')
            if i < n_rows:
                fts.add_edge(cur_cell, f'c_{i+1}{j}')
            if j > 1:
                fts.add_edge(cur_cell, f'c_{i}{j-1}')
            if j < n_cols:
                fts.add_edge(cur_cell, f'c_{i}{j+1}')

    # print("States: ", fts.nodes)
    # print("Transitions: ", list(fts.edges))

    init_state = 'c_11'
    labels = {
        'c_12': 'b & !d & !g',
        'c_23': 'g & !b & !d',
        'c_13': 'd & !b & !g',
        'c_31': 'd & !b & !g'
    }
    # labels = {
    #     'c_11': 'b & !d',
    #     'c_13': '!b & d'
    # }
    # print(labels)

    ltl = spot.formula('GF b & G(b -> X !b U F g) & G!(b & g) & G !d')
    # ltl = spot.formula('GF b & G! d & GF g & G!(b & g)')
    buchi_aut = spot.translate(ltl, 'Buchi', 'high', 'Deterministic', 'state-based')
    bdd_dict = buchi_aut.get_dict()

        
    product_aut, init_states, accepting_states = construct_product_automaton(fts, buchi_aut, labels)
    # print("Product nodes: ", product_aut.nodes)
    # print("Product transitions: ", list(product_aut.edges))
    # print("Accepted states: ", accepting_states)

    accepting_run = find_accepting_run(product_aut, init_states, accepting_states)

    if accepting_run:
        prefix, suffix = accepting_run  # Unpack the returned tuple
        # Project the prefix and suffix onto the original FTS
        fts_prefix = [state[0] for state in prefix]
        fts_suffix = [state[0] for state in suffix]
        
        print("Found satisfying run in FTS:")
        print("Prefix:", fts_prefix)
        print("Suffix:", fts_suffix)

        # Buchi sequence
        buchi_prefix = [state[1] for state in prefix]
        buchi_suffix = [state[1] for state in suffix]
        
        print("Found satisfying run in Buchi:")
        print("Prefix:", buchi_prefix)
        print("Suffix:", buchi_suffix)

        if prefix is None:
            ltl_sequence = np.array(fts_suffix)
        else:
            ltl_sequence = np.concatenate((np.array(fts_prefix), np.array(fts_suffix)))

        with open('ltl_sequence.pkl', 'wb') as f:
            pickle.dump(ltl_sequence, f)
            print("LTL sequence saved")

        with open('ltl_sequence.pkl', 'rb') as f:
            loaded_data = pickle.load(f)
        print(loaded_data)
        
    else:
        print("No satisfying run found")


    






