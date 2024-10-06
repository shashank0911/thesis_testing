import networkx as nx
import spot
import pygraphviz as pgv

n_rows = 2
n_cols = 3


def create_fts():
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
    
    return fts

fts = create_fts()
init_state = 'c_11'
# print("States: ", fts.nodes)
# print("Transitions: ", list(fts.edges))


ltl1 = spot.formula('GF B & G !D & G(B -> X !B U F G1)')
ltl2 = spot.formula('GF gather & GF upload')
ltl3 = spot.formula('GF B & G! D')
ltl4 = spot.formula('GF B & GF G1 & G !D & G(B -> X !B U F G1)')
print(ltl1)
print(ltl2)
print(ltl3)

aut1 = spot.translate(ltl1, 'Buchi', 'state-based', 'high')
dot_str = aut1.to_str('dot')
g = pgv.AGraph(string=dot_str)
g.draw('output.png', prog='dot', format='png')

aut2 = spot.translate(ltl2, 'Buchi', 'state-based', 'high')
# aut2 = spot.translate(ltl1, 'parity', 'state-based', 'complete', 'high')
dot_str = aut2.to_str('dot')
g = pgv.AGraph(string=dot_str)
g.draw('output2.png', prog='dot', format='png')

print(spot.are_equivalent(aut1, aut2))

# aut2 = spot.translate(ltl3, 'low')
# dot_str = aut2.to_str('dot')
# g = pgv.AGraph(string=dot_str)
# g.draw('output3.png', prog='dot', format='png')
