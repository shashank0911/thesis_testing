n = 3;  % Number of dimensions
r = 1;  % Radius of the ball

% Create a grid of points in 3D
theta = linspace(0, pi, 10);
phi = linspace(0, 2*pi, 20);
[Theta, Phi] = ndgrid(theta, phi);

X = r * sin(Theta) .* cos(Phi);
Y = r * sin(Theta) .* sin(Phi);
Z = r * cos(Theta);

% Flatten and create a Polyhedron
V = [X(:), Y(:), Z(:)];
ball = Polyhedron('V', V);

% Plot the ball
figure;
plot(ball, 'color', 'blue', 'alpha', 0.5);
title(sprintf('3D Ball of Radius %g', r));
