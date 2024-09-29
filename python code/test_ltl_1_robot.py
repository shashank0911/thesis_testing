from __future__ import print_function

# import logging

from tulip import transys, spec, synth
# from tulip.transys import machines

# import numpy as np

sys = transys.FTS()

cells = ['c_{}{}'.format(i, j) for i in range (1, 6) for j in range (1,6)]
sys.states.add_from(cells)
sys.states.initial.add('c_11')

sys.atomic_propositions.add_from({'B', 'G1', 'G2', 'G3', 'D'})
sys.states.add('c_11', ap={'B'})
sys.states.add('c_15', ap={'D'})
sys.states.add('c_23', ap={'D'})
sys.states.add('c_25', ap={'G2'})
sys.states.add('c_31', ap={'D'})
sys.states.add('c_51', ap={'G1'})
sys.states.add('c_53', ap={'D'})
sys.states.add('c_54', ap={'G3'})

for i in range(1, 6):
    for j in range (1, 6):
        cur_cell = 'c_{}{}'.format(i, j)

        if i > 1:
            sys.transitions.add(cur_cell, 'c_{}{}'.format(i-1, j))
        if i < 5:
            sys.transitions.add(cur_cell, 'c_{}{}'.format(i+1, j))
        if j > 1:
            sys.transitions.add(cur_cell, 'c_{}{}'.format(i, j-1))
        if j < 5:
            sys.transitions.add(cur_cell, 'c_{}{}'.format(i, j+1))

ltl_spec = spec.GRSpec()
ltl_spec.env_vars = {}
ltl_spec.env_init = {}
ltl_spec.env_prog = {}
ltl_spec.env_safety = {}  

ltl_spec.sys_vars = {}      
ltl_spec.sys_init = {'B'}
ltl_spec.sys_safety = {'!D'}
ltl_spec.sys_prog = {'B', 'G1', 'G2', 'G3'}

ctrl = synth.synthesize(specs=ltl_spec, sys=sys)

if ctrl is None:
    print("No controller found that satisfies the given LTL formula.")
else:
    print("Controller found!")
    print("Strategy transition:")
    for state_from, state_to in ctrl.edges():
        print(f"Transition from {state_from} to {state_to}")