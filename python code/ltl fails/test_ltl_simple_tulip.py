from __future__ import print_function

# import logging

from tulip import transys, spec, synth
# from tulip.transys import machines

# import numpy as np

sys = transys.FTS()
n=2
m=3

cells = ['c_{}{}'.format(i, j) for i in range (1, n+1) for j in range (1, m+1)]
sys.states.add_from(cells)
sys.states.initial.add('c_11')

sys.atomic_propositions.add_from({'B', 'G', 'D'})
sys.states.add('c_11', ap={'B'})
sys.states.add('c_21', ap={'D'})
sys.states.add('c_13', ap={'G'})

for i in range(1, n+1):
    for j in range (1, m+1):
        cur_cell = 'c_{}{}'.format(i, j)

        if i > 1:
            sys.transitions.add(cur_cell, 'c_{}{}'.format(i-1, j))
        if i < n:
            sys.transitions.add(cur_cell, 'c_{}{}'.format(i+1, j))
        if j > 1:
            sys.transitions.add(cur_cell, 'c_{}{}'.format(i, j-1))
        if j < m:
            sys.transitions.add(cur_cell, 'c_{}{}'.format(i, j+1))

ltl_spec = spec.GRSpec()
ltl_spec.env_vars = {}
ltl_spec.env_init = {}
ltl_spec.env_prog = {}
ltl_spec.env_safety = {}  

ltl_spec.sys_vars = {'B_to_G': 'boolean'}      
ltl_spec.sys_init = {'B', 'B_to_G'}
ltl_spec.sys_safety = {'!D', '(X (B_to_G) <-> G) || (B_to_G && !B)'}
# ltl_spec.sys_safety |= {'(X (B_to_G) <-> G) || (B_to_G && !B)'}
ltl_spec.sys_prog = {'B'}
ltl_spec.moore = True
ltl_spec.qinit = r'\E \A'
ctrl = synth.synthesize(specs=ltl_spec, sys=sys)

if ctrl is None:
    print("No controller found that satisfies the given LTL formula.")
else:
    print("Controller found!")
    print("Strategy transition:")
    for state_from, state_to in ctrl.transitions():
        print(f"Transition from {state_from} to {state_to}")