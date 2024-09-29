%% 
T = 0.1;
A = [eye(2) T*eye(2);
     zeros(2) eye(2)];
B = [zeros(2);
     eye(2)];
sys = LTISystem('A', A, 'B', B);
sys.x.max = [60; 60; 30; 30];
sys.x.min = [0; 0; -30; -30];
sys.u.max = [5; 5];
sys.u.min = [-5; -5];
ogSet = Polyhedron('lb', [0; 0], 'ub', [60; 60]);
extraSet = Polyhedron('lb', [-5; -5], 'ub', [5; 5]);
velSet = Polyhedron('lb', [-30, -30], 'ub', [30, 30]);
inSet = Polyhedron('lb', [-2, -2], 'ub', [2, 2]);
% blankSet = Polyhedron([zeros(8,4)])
minksum = ogSet + extraSet;
minkdiff = ogSet - extraSet;
minksum.plot(color='red')
hold on
ogSet.plot(color='blue')
figure
ogSet.plot(color='blue')
hold on
minkdiff.plot(color='red')

H = blkdiag(minkdiff.A, velSet.A);
h = [minkdiff.b; velSet.b];
newSet = Polyhedron(H, h);
conInvSet = sys.invariantSet('X', newSet, 'U', inSet);
figure
ogSet.plot(color='blue')
hold on
conInvSet.projection([1, 2]).plot(color='red')

%%
% modifiedSet = 

figure
conInvSet = sys.invariantSet();
polyProj = conInvSet.projection([1, 2]);
polyProj.plot()

%%
A = [1.5 0; 1 -1.5];
B = [1; 0];
sys = LTISystem('A', A, 'B', B);
sys.x.max = [10; 10];
sys.x.min = [-10; -10];
sys.u.max = 5;
sys.u.min = -5;
conInvSet = sys.invariantSet;
H = [eye(2); -eye(2)];
h = 10*ones(4,1);
Hu = [1; -1];
hu = [5; 5];
ogSet = Polyhedron('H', [H, h]);
uSet = Polyhedron('H', [Hu, hu]);
% ogSet.plot(color='blue')
% hold on
% conInvSet.plot(color='red')
preX = Polyhedron([H*A H*B; zeros(size(Hu,1), size(H*A,2)) Hu], [h; hu]);
preXproj = preX.projection(1:2);
inters = intersect(preXproj, ogSet);
inters.H
ogSet.b(1)
