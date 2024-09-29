import matlab.engine
import numpy as np
from polytope import Polytope, is_subset, projection

class EnvCell:
    def __init__(self, i, j, xmin, xmax, ymin, ymax, sys, 
                 velocityConstraints, inputConstraints, delta, eng):
        self.state = f'c_{i}{j}'
        self.center = np.array([xmin/2 + xmax/2, ymin/2 + ymax/2])
        H = np.vstack([np.eye(2), -np.eye(2)])
        h = np.array([xmax, ymax, -xmin, -ymin])
        self.positionConstraints = Polytope(H, h)
        self.controlInvariantSet = None
        self.controlInvariantPosSet = None

        self.sys = sys

        self.velocityConstraints = velocityConstraints
        self.inputConstraints = inputConstraints
        self.delta = delta

        self.find_control_invariant_set(eng)

    def find_control_invariant_set(self, eng):
        
        # positionConstraints_A = matlab.double(self.positionConstraints.A.tolist())
        # positionConstraints_b = matlab.double(self.positionConstraints.b.reshape(-1, 1).tolist())
        # eng.eval("positionConstraints = Polyhedron('H', {x}, 'h', {y});".format(x=positionConstraints_A, y=positionConstraints_b), nargout=0)
        eng.workspace['positionConstraints_A'] = matlab.double(self.positionConstraints.A.tolist())
        eng.workspace['positionConstraints_b'] = matlab.double(self.positionConstraints.b.reshape(-1, 1).tolist())
        eng.eval("positionConstraints = Polyhedron(positionConstraints_A, positionConstraints_b);", nargout=0)
        # print(self.positionConstraints.A)
        # print(self.positionConstraints.b)
        # print(positionConstraints_A)
        # print(positionConstraints_b)
        # eng.disp(positionConstraints, nargout=0)

        # velocityConstraints_A = matlab.double(self.velocityConstraints.A.tolist())
        # velocityConstraints_b = matlab.double(self.velocityConstraints.b.reshape(-1, 1).tolist())
        eng.workspace['velocityConstraints_A'] = matlab.double(self.velocityConstraints.A.tolist())
        eng.workspace['velocityConstraints_b'] = matlab.double(self.velocityConstraints.b.reshape(-1, 1).tolist())
        eng.eval("velocityConstraints = Polyhedron(velocityConstraints_A, velocityConstraints_b);", nargout=0)

        # inputConstraints_A = matlab.double(self.inputConstraints.A.tolist())
        # inputConstraints_b = matlab.double(self.inputConstraints.b.reshape(-1, 1).tolist())
        # eng.eval("inputConstraints = Polyhedron({x}, {y});".format(x=inputConstraints_A, y=inputConstraints_b), nargout=0)
        eng.workspace['inputConstraints_A'] = matlab.double(self.inputConstraints.A.tolist())
        eng.workspace['inputConstraints_b'] = matlab.double(self.inputConstraints.b.reshape(-1, 1).tolist())
        eng.eval("inputConstraints = Polyhedron(inputConstraints_A, inputConstraints_b);", nargout=0)
        # inputConstraints = eng.Polyhedron(inputConstraints_A, inputConstraints_b)

        eng.workspace['A'] = matlab.double(self.sys.A.tolist())
        eng.workspace['B'] = matlab.double(self.sys.B.tolist())
        eng.eval("LTIsys = LTISystem('A', A, 'B', B);", nargout=0)

        eng.workspace['d'] = matlab.double([self.delta])
        if self.delta == 0:
            
            eng.eval("H = blkdiag(positionConstraints.A, velocityConstraints.A);", nargout=0)
            eng.eval("h = [positionConstraints.b; velocityConstraints.b];", nargout=0)
            eng.eval("stateConstraints = Polyhedron(H, h);", nargout=0)
            eng.eval("controlInvariantSet = LTIsys.invariantSet('X', stateConstraints, 'U', inputConstraints);", nargout=0)
            # H = eng.blkdiag(positionConstraints_A, velocityConstraints_A)
            # h = eng.vertcat(positionConstraints_b, velocityConstraints_b)
            # eng.stateConstraints = eng.Polyhedron(H, h)
            # controlInvariantSet = LTIsys.invariantSet('X', stateConstraints, 'U', inputConstraints)
        
        elif self.delta > 0:
            eng.eval("Hd = [eye(2); -eye(2)];", nargout=0)
            eng.eval("hd = [d; d; d; d];", nargout=0)
            eng.eval("Bd = Polyhedron(Hd, hd);", nargout=0)
            eng.eval("modifiedConstraintSet = positionConstraints - Bd;", nargout=0)
            # ub = np.array([self.delta, self.delta])
            # lb = -ub
            # ubmat = matlab.double(ub.reshape(-1, 1).tolist())
            # lbmat = matlab.double(lb.reshape(-1, 1).tolist())
            # Bd = eng.Polyhedron('lb', lbmat, 'ub', ubmat)
            # modifiedConstraintSet = eng.minus(positionConstraints, Bd)
            # eng.disp(modifiedConstraintSet, nargout=0)

            # modifiedConstraintSet_A = eng.eval('modifiedConstraintSet.A', nargout=1)
            # modifiedConstraintSet_b = eng.eval('modifiedConstraintSet.b', nargout=1)
            eng.eval("H = blkdiag(modifiedConstraintSet.A, velocityConstraints.A);", nargout=0)
            eng.eval("h = [modifiedConstraintSet.b; velocityConstraints.b];", nargout=0)
            eng.eval("stateConstraints = Polyhedron(H, h);", nargout=0)
            eng.eval("controlInvariantSet = LTIsys.invariantSet('X', stateConstraints, 'U', inputConstraints);", nargout=0)
            eng.eval("controlInvariantPosSet = controlInvariantSet.projection(1:2);", nargout=0)

            # print("conInvSet matlab")
            # eng.eval("controlInvariantPosSet.A", nargout=0)
            # eng.eval("controlInvariantPosSet.b", nargout=0)
            # eng.eval("positionConstraints.contains(controlInvariantSet.projection(1:2))", nargout=0)
            # print("conInvSet matlab")
            # eng.eval("controlInvariantSet.A", nargout=0)
            # eng.eval("controlInvariantSet.b", nargout=0)
            
            # H = eng.blkdiag(modifiedConstraintSet_A, velocityConstraints_A)
            # h = eng.vertcat(modifiedConstraintSet_b, velocityConstraints_b)
            # stateConstraints = eng.Polyhedron(H, h)
            # controlInvariantSet = LTIsys.invariantSet('X', stateConstraints, 'U', inputConstraints)

        controlInvariantSet_A = np.array(eng.eval("controlInvariantSet.A;"))
        controlInvariantSet_b = np.array(eng.eval("controlInvariantSet.b;"))
        self.controlInvariantSet = Polytope(controlInvariantSet_A, controlInvariantSet_b, normalize=False)
        # temp = projection(self.controlInvariantSet,[0,1])
        # temp = self.controlInvariantSet.project([0,1])
        # print("conInvSet python:\n")
        # print(self.controlInvariantSet)
        # print(temp.A)
        # print(temp.b)
        controlInvariantPosSet_A = np.array(eng.eval("controlInvariantPosSet.A;"))
        controlInvariantPosSet_b = np.array(eng.eval("controlInvariantPosSet.b;"))
        self.controlInvariantPosSet = Polytope(controlInvariantPosSet_A, controlInvariantPosSet_b)
        # print("conInvSet python:\n")
        # print(self.controlInvariantPosSet)
        # print(np.shape(self.controlInvariantPosSet.b))

        # if is_subset(temp, self.positionConstraints):
        #     print("Control invariant set is inside position set for ", self.state)
        # else:
        #     print("Not inside!")
        # print(np.shape(self.controlInvariantSet.A))
        # print(np.shape(self.controlInvariantSet.b))
        eng.eval("clear", nargout=0)





        