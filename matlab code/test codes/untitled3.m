% A = [eye(2); -eye(2)];
% b1 = [5; 5; 0; 0];
% b2 = [10; 5; -5; 0];
% P(1) = Polyhedron(A, b1);
% P(2) = Polyhedron(A, b2);
% U = PolyUnion(P);
% H = U.convexHull;
% figure
% P(1).plot()
% hold on
% P(2).plot()
% figure
% H.plot()


lb = [0; 0];
ub = [60; 60];
P1 = Polyhedron('lb', lb, 'ub', ub);
P1.H
lb2 = [-5; -5];
ub2 = [5; 5];
H2 = blkdiag(P1.A, [eye(2); -eye(2)]);
h2 = [P1.b; ub2; -lb2];
P2 = Polyhedron(H2, h2);
P2.H
p1 = projection(P2, 1:2);
p2 = projection(P2, 3:4);
p1.plot()
figure
p2.plot()
