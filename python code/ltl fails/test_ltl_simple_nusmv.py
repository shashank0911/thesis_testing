# from pynusmv import glob, mc
# from pynusmv.init import init_nusmv, deinit_nusmv


# def create_model():
#     model_text ="""
#     MODULE main
#     VAR
#         state: {c_11, c_12, c_13, c_21, c_22, c_23};

#     DEFINE
#         base := state = c_11;
#         goal := state = c_23;
#         danger := state = c_13;

#     ASSIGN
#         init(state) := c_11;

#         next(state) := case
#             state = c_11 : {c_12, c_21};
#             state = c_12 : {c_11, c_13, c_22};
#             state = c_13 : {c_12, c_23};
#             state = c_21 : {c_11, c_22};
#             state = c_22 : {c_21, c_12, c_23};
#             state = c_23 : {c_22, c_13};
#         esac;

#     LTLSPEC
#         G F (base) & G(base -> X(!base U F goal)) & G(!danger)

#     """
#     return model_text


# def main():
#     init_nusmv()
#     model_text = create_model()
#     glob.load_from_string(model_text)
#     glob.compute_model()

#     spec = glob.prop_database()[0]
#     fsm = glob.prop_database().master.bddFsm
#     result = mc.check_ltl_spec(spec)

#     if result:
#         print("Specification is satisfied.")
#         print("A valid path:")
#         path = mc.explain(spec, mc.Explain.WITNESS)
#         for s in path:
#             print(f"State: {s['state']}")
#     else:
#         print("Specification is not satisfied.")
#         print("A counterexample:")
#         path = mc.explain(spec, mc.Explain.COUNTEREXAMPLE)
#         for s in path:
#             print(f"State: {s['state']}")

#     deinit_nusmv()

# if __name__ == "__main__":
#     main()




import pynusmv
pynusmv.init.init_nusmv()
pynusmv.glob.load_from_file("model.smv")
pynusmv.glob.compute_model()
fsm = pynusmv.glob.prop_database().master.bddFsm
print(fsm)
prop = pynusmv.glob.prop_database()[0]
print(prop)
spec = prop.expr
print(spec)
val = pynusmv.mc.check_ltl_spec(spec)
print(val)
# explanation = pynusmv.mc.explain(fsm, fsm.init, spec)
# for state, inputs in zip(explanation[::2], explanation[1::2]):
#     if state == explanation[-1]:
#         print("-- Loop starts here")
#     print(state.get_str_values())
#     print(inputs.get_str_values())
# if val:
#     print("LTL specification is satisfied.")
# else:
#     print("LTL specification is not satisfied. Generating counterexample...")

#     # Generate counterexample
#     path = pynusmv.mc.explain(prop)
#     print("Counterexample path:")
#     for state in path:
#         print(f"State: {state['state']}")