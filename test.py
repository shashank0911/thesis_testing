import numpy as np
# import pypolycontain as pp
import polytope as pc
import matplotlib.pyplot as plt

H = np.array([[1, 0],
             [0, 1],
             [-1, 0],
             [0, -1]])
h = np.array([10, 10, 10, 10])
p = pc.Polytope(H, h)
# print("Polytope p:\n", p)
# p = pp.H_polytope(H, h)
# print("Polytope p:\n", p.H, '\n\n', p.h)

H1 = np.array([[1, 0],
               [1, -0.5],
               [-1, 0],
               [-1, 0.5]])
h1 = np.array([20, 10, 20, 10])
p1 = pc.Polytope(H1, h1)
# print("Polytope p1:\n", p1)
# p1 = pp.H_polytope(H1, h1)
# print("\nPolytope p1:\n", p1.H, '\n\n', p1.h)

p2 = p.intersect(p1)
# print("Polytope p2:\n", p2)


# fig, ax = plt.subplots(figsize=(10, 8))
# p.plot(ax, color='lightblue', alpha=0.3)
# p2.plot(ax, color='lightgreen', alpha=0.3)

# ax.grid(True, which='both', linestyle='--', linewidth=0.5)
# ax.axis('equal')
# ax.grid(True)
# plt.show()