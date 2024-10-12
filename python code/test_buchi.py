import spot
import pygraphviz as pgv

ltl1 = spot.formula('GF b & G(b -> X !b U F g) & G!(b & g) & G !d')
ltl2 = spot.formula('b & GF b & G! d & GF g & G!(b & g)')
ltl3 = spot.formula('GF base & G! danger & GF goal & G!(base & goal)')
ltl4 = spot.formula('GF B & GF G1 & G !D & G(B -> X !B U F G1)')
ltl1 = spot.formula('GF b & GF g & G!(b & g)')
# ltl2 = spot.formula('GF b & G(b -> X !b U F g) & G!(b & g)')
print(ltl1)
# print(ltl2)
# print(ltl3)

ltl_a = spot.formula('GF b_p & GF g_p & G!(b_p & g_p)')
ltl_b = spot.formula('GF b_q & GF g_q & G!(b_q & g_q)')
ltl_combined = spot.formula(f"({ltl_a}) & ({ltl_b})")
print(ltl_combined)

aut1 = spot.translate(ltl1, 'Buchi', 'high', 'Deterministic', 'state-based')
dot_str = aut1.to_str('dot')
g = pgv.AGraph(string=dot_str)
g.draw('output.png', prog='dot', format='png')

aut2 = spot.translate(ltl_combined, 'Buchi', 'Deterministic', 'high', 'state-based')
dot_str = aut2.to_str('dot')
g = pgv.AGraph(string=dot_str)
g.draw('output2.png', prog='dot', format='png')

ltlb = spot.formula('b & GF b & G !d & G(b -> X !b U F g) & G!(b & g) & GF g')
autb = spot.translate(ltlb, 'Buchi', 'state-based', 'high')
# print(spot.are_equivalent(aut1, autb))

# Printing transitions as strings
# bdd_dict = aut1.get_dict()
# for state in range(aut1.num_states()):
#     print(f"State:{state}")
#     for transition in aut1.out(state):
#         condition = transition.cond
#         cond_act = spot.bdd_format_formula(bdd_dict, condition)
#         print(f" Transition from state {transition.src} to state {transition.dst} with condition {cond_act}")



# aut2 = spot.translate(ltl3, 'low')
# dot_str = aut2.to_str('dot')
# g = pgv.AGraph(string=dot_str)
# g.draw('output3.png', prog='dot', format='png')



# run = aut1.accepting_run()
# print(run)
# word = spot.twa_word(run)
# print(word)
