import networkx as nx
import spot
import pygraphviz as pgv

n_rows = 2
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
    'c_11': 'base',
    'c_13': 'goal',
    'c_23': 'danger'
}

ltl = spot.formula('GF base & GF goal & G! danger')
buchi_aut = spot.translate(ltl, 'Buchi', 'state-based', 'high')

# print(buchi_aut.num_states())


def construct_product_automaton(fts, buchi_aut, labels):
    product = nx.DiGraph()
    bdd_dict = buchi_aut.get_dict()
    
    # 1. Define states of the product automaton
    for fts_state in fts.nodes():
        for buchi_state in range(buchi_aut.num_states()):
            product.add_node((fts_state, buchi_state))
    
    # 2. Determine initial states
    initial_states = [(init_state, buchi_aut.get_init_state_number())]
    
    # 3. Create transition relation  
    for fts_state in fts.nodes():
        for buchi_state in range(buchi_aut.num_states()):
            for fts_next in fts.successors(fts_state):
                # Create the label for the current FTS state
                state_label = spot.bdd_dict()
                
                
                for buchi_next in buchi_aut.out(buchi_state):
                    if spot.bdd_implies(bdd_dict.bdd_encode(state_label), buchi_next.cond):
                    # if buchi_aut.get_dict().is_true(buchi_next.cond) or (state_label & buchi_next.cond):
                        product.add_edge((fts_state, buchi_state), (fts_next, buchi_next.dst))
                    
                    # buchi_label = str(buchi_next.cond)
                    # label_cond = labels.get(fts_state, None)
                    # print(label_cond)
                    # if label_cond and label_cond in buchi_label:
                    #     product.add_edge((fts_state, buchi_state), (fts_next, buchi_next.dst))
    
    # 4. Identify accepting states
    accepting_states = [(fts_state, buchi_state) for fts_state in fts.nodes() 
                        for buchi_state in range(buchi_aut.num_states()) 
                        if buchi_aut.state_is_accepting(buchi_state)]
    
    return product, initial_states, accepting_states

# Construct the product automaton
product_aut, init_states, accept_states = construct_product_automaton(fts, buchi_aut, labels)

# print(product_aut.nodes)
# print(list(product_aut.edges))

# Now you can use this product automaton to find accepting runs
# For example, using NetworkX to find a path that visits accepting states:
def find_accepting_run(product_aut, init_states, accept_states):
    # Simple implementation: find a path to an accepting state, then a cycle
    start = init_states[0]
    for accept in accept_states:
        try:
            path_to_accept = nx.shortest_path(product_aut, start, accept)
            cycle = nx.find_cycle(product_aut, accept)
            return path_to_accept + [state for state, _ in cycle[:-1]]
        except (nx.NetworkXNoPath, nx.NetworkXNoCycle):
            continue
    return None

accepting_run = find_accepting_run(product_aut, init_states, accept_states)

if accepting_run:
    # Project the run onto the original FTS
    fts_run = [state[0] for state in accepting_run]
    print("Found satisfying run in FTS:", fts_run)
else:
    print("No satisfying run found")
















# return product_states, product_transitions
    

# fts = create_fts()



# ltl = spot.formula('GF base & GF goal & G! danger')
# buchi_aut = spot.translate(ltl, 'Buchi', 'state-based', 'high')

# prod_states, prod_trans = product_automaton(fts, buchi_aut, labels)
