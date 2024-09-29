# import matlab.engine
import numpy as np
from polytope import Polytope

# eng = matlab.engine.start_matlab()
# eng.run('../matlab code/tbxmanager/startup.m', nargout=0)

vmax = np.array([30, 30])
vmin = np.array([-30, -30])
A = np.vstack((np.eye(2), -np.eye(2)))
# print(len(A))
# print(len(A[0]))
b = np.concatenate([vmax, -vmin])
# print(np.shape(b))
b = b.reshape(-1, 1)
# print(np.shape(b))
# print(A)
# print(b)
velocityConstraints = Polytope(A, b)
# print(np.shape(velocityConstraints.b))
# velocityConstraints_A = matlab.double(velocityConstraints.A.tolist())
# velocityConstraints_b = matlab.double(velocityConstraints.b.reshape(-1,1).tolist())
# # print(velocityConstraints_A)
# # print(velocityConstraints_b)
# eng.workspace['H'] = velocityConstraints_A
# eng.workspace['h'] = velocityConstraints_b
# # eng.eval("disp(H)", nargout=0)
# # eng.eval("disp(h)", nargout=0)
# eng.eval("velC = Polyhedron(H, h);", nargout=0)
# # eng.eval("disp(velC.A)", nargout=0)
# bx = np.array(eng.eval("velC.b")).flatten()
# print(np.array(bx))
# # print(len(Ax))
# # print(len(Ax[0]))
# # eng.eval("disp(velC.H)", nargout=0)
# # eng.eval("disp(velC.b)", nargout=0)
iph = np.zeros((2,50))
# print(np.shape(iph[:,0]))


x = 1
y = 2
z1 = np.array([[x, y]])
z2 = np.array([[x], [y]])
print(z1[0][1])
print(z2[1][0])


