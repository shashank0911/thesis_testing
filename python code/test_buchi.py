import spot
import pygraphviz as pgv

ltl1 = spot.formula('GF B & G !D & G(B -> X !B U F G1)')
ltl2 = spot.formula('GF gather & GF upload')
ltl3 = spot.formula('GF base & G! danger & GF goal')
ltl4 = spot.formula('GF B & GF G1 & G !D & G(B -> X !B U F G1)')
print(ltl1)
print(ltl2)
print(ltl3)

aut1 = spot.translate(ltl3, 'Buchi', 'state-based', 'high')
dot_str = aut1.to_str('dot')
g = pgv.AGraph(string=dot_str)
g.draw('output.png', prog='dot', format='png')

aut2 = spot.translate(ltl2, 'Buchi', 'SBAcc', 'high')
# aut2 = spot.translate(ltl1, 'parity', 'state-based', 'complete', 'high')
dot_str = aut2.to_str('dot')
g = pgv.AGraph(string=dot_str)
g.draw('output2.png', prog='dot', format='png')

print(spot.are_equivalent(aut1, aut2))

bdd_dict = aut1.get_dict()
for state in range(aut1.num_states()):
    print(f"State:{state}")
    for transition in aut1.out(state):
        condition = transition.cond
        cond_act = spot.bdd_format_formula(bdd_dict, condition)
        print(f" Transition to state {transition.dst} with condition {cond_act}")

# aut2 = spot.translate(ltl3, 'low')
# dot_str = aut2.to_str('dot')
# g = pgv.AGraph(string=dot_str)
# g.draw('output3.png', prog='dot', format='png')
