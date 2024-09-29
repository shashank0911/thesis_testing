% Number of rows and columns of cells
n = 3;
m = 3;
% Cell dictionary with each entry as an EnvCell object
% cellArray = cell(n, m);
cellDict = dictionary;
% Order of system
T = 1;

env_xmin = 0;
env_xmax = 90;
env_ymin = 0;
env_ymax = 90;

delta = 3;

LTI.A = [eye(2), T*eye(2); zeros(2) eye(2)];
LTI.B = [zeros(2); eye(2)];
LTI.C = [1 0 0 0;
     0 1 0 0];
sys = LTISystem('A', LTI.A, 'B', LTI.B, 'C', LTI.C);


dim.nx = 4;
dim.nu = 2;
dim.ny = 2;
dim.N = 20;
dim.simsteps = 50;

weight.Q = diag([10, 10, 1, 1]);
weight.R = diag([1, 1]);
weight.Qf = weight.Q;

vmax = [30; 30];
vmin = [-30; -30];
dvmax = [5; 5];
dvmin = [-5; -5];
constraints.velocity = Polyhedron('lb', vmin, 'ub', vmax);
constraints.input = Polyhedron('lb', dvmin, 'ub', dvmax);
constraints.delta = delta;

xbound = linspace(env_xmin, env_xmax, n+1);
ybound = linspace(env_ymax, env_ymin, m+1);

for i = 1:n
    for j = 1:m
        xmin = xbound(j);
        xmax = xbound(j+1);
        ymax = ybound(i);
        ymin = ybound(i+1);

        % cellArray{i, j} = EnvCell(i, j, n_sys, xmin, xmax, ymin, ymax);
        keyString = sprintf('c_%d%d', i, j);
        cellDict(keyString) = EnvCell(i, j, xmin, xmax, ymin, ymax, sys, constraints);
        
    end
end

idList = ["1234", "2468", "1357"];
x01List = [5; 5; 0; 0];
x02List = [75; 80; 0; 0];
x03List = [5; 40; 0; 0];
x0List = [x01List, x02List, x03List];

% idList = ["1234"];
% x0List = [5; 5; 0; 0];


robots = Robots(idList, x0List, cellDict);
robots = robots.initialiseMidLevelSystem(sys, dim);

% Map motion plan to each robot here
% motionPlan = ["c_31", "c_32", "c_22", "c_23", "c_33", "c_32", "c_22", "c_12", "c_11", "c_21"];

motionPlan = ["c_31", "c_32", "c_22"];
robots.robotList(1).topSystem = TopLevelSystem(motionPlan);
motionPlan = ["c_13", "c_12", "c_11"];
robots.robotList(2).topSystem = TopLevelSystem(motionPlan);
motionPlan = ["c_21", "c_22", "c_23"];
robots.robotList(3).topSystem = TopLevelSystem(motionPlan);

% [sequencePair, robots.robotList(1).topSystem] = robots.robotList(1).topSystem.getSequencePair();
% highLevelInput = robots.findHighLevelInput(sequencePair);
robots = robots.initialiseMidLevelControl(weight, constraints.velocity, constraints.input);

% robots.robotList(1).midControl = robots.robotList(1).midControl.getConstraints(highLevelInput);
% figure
% robots.robotList(1).midControl.positionConstraints.plot()
% xfCheby = highLevelInput{2}.constraintSet.chebyCenter();
% robots.robotList(1).midSystem.xf = xfCheby.x;


execTimeArr = zeros(dim.simsteps,1);
for t = 1:dim.simsteps
    tic
    fprintf("Loop number:%d\n", t)
    for i = 1:robots.numRobots
        robots.robotList(i).midSystem.t = t;
        if robots.robotList(i).midControl.terminate == true
            [sequencePair, robots.robotList(i).topSystem] = robots.robotList(i).topSystem.getSequencePair();
            if robots.robotList(i).topSystem.terminate == true
                robots.robotList(i).midSystem.stateHistory(:, t+1) = robots.robotList(i).midSystem.xc;
                robots.robotList(i).midSystem.inputHistory(:, t) = robots.robotList(i).midSystem.uc(1:dim.nu);
                continue
            else 
                highLevelInput = robots.findHighLevelInput(sequencePair);
                robots.robotList(i).midControl = robots.robotList(i).midControl.getConstraints(highLevelInput);
                % figure
                % robots.robotList(i).midControl.positionConstraints.plot()
                xfCheby = highLevelInput{2}.constraintSet.chebyCenter();
                robots.robotList(i).midSystem.xf = xfCheby.x;
                % combinedStateConstraints.A = blkdiag(kron(eye(dim.N), robots.robotList(i).midControl.stateConstraints.A), robots.robotList(i).midControl.terminalConstraints.A);
                % combinedStateConstraints.b = [repmat(robots.robotList(i).midControl.stateConstraints.b, dim.N, 1); robots.robotList(i).midControl.terminalConstraints.b];
                % combinedInputConstraints.A = kron(eye(dim.N), robots.robotList(i).midControl.inputConstraints.A);
                % combinedInputConstraints.b = repmat(robots.robotList(i).midControl.inputConstraints.b, dim.N, 1);
                % robots.robotList(i).midControl.combinedStateConstraints = combinedStateConstraints;
                % robots.robotList(i).midControl.combinedInputConstraints = combinedInputConstraints;
                robots.robotList(i).midControl.terminate = false;
            end
        end

        % if robots.robotList(i).topSystem.terminate == true
        %     % fprintf("here?")
        %     continue
        % else 
        %     if robots.robotList(i).midControl.terminate == true
        %         [sequencePair, robots.robotList(i).topSystem] = robots.robotList(i).topSystem.getSequencePair();
        %         highLevelInput = robots.findHighLevelInput(sequencePair);
        %         robots.robotList(i).midControl = robots.robotList(i).midControl.getConstraints(highLevelInput);
        %         xfCheby = highLevelInput{2}.constraintSet.chebyCenter();
        %         robots.robotList(i).midSystem.xf = xfCheby.x;
        %         robots.robotList(i).midControl.terminate = false;
        %     end
        [robots.robotList(i).midSystem, robots.robotList(i).midControl] = robots.robotList(i).midSystem.getMPC(robots.robotList(i).midControl);
        % robots.robotList(i).midSystem = midSystem;
        % robots.robotList(i).midControl = midControl;
        
        % end
    end
    elapsedTime = toc;
    execTimeArr(t) = elapsedTime;
end







%% Plotting

x_data = robots.robotList(1).midSystem.stateHistory(1,:);
y_data = robots.robotList(1).midSystem.stateHistory(2,:);
figure
hold on
plot(x_data, y_data, '-b.', 'LineWidth', 2, 'MarkerSize', 15);

x_data = robots.robotList(2).midSystem.stateHistory(1,:);
y_data = robots.robotList(2).midSystem.stateHistory(2,:);
plot(x_data, y_data, '-r.', 'LineWidth', 2, 'MarkerSize', 15);

x_data = robots.robotList(3).midSystem.stateHistory(1,:);
y_data = robots.robotList(3).midSystem.stateHistory(2,:);
plot(x_data, y_data, '-g.', 'LineWidth', 2, 'MarkerSize', 15);

keyList = keys(cellDict);
for i = 1:length(keyList)
    constSet = cellDict(keyList(i)).constraintSet;
    constSet.plot('color', [0.8, 0.8, 0.8], 'alpha', 0.3, 'LineWidth', 1.5, 'EdgeColor', 'k');
    invSet = cellDict(keyList(i)).controlInvariantSet.projection(1:2);
    invSet.plot('color', [0.6, 0.6, 0.6], 'alpha', 0.3, 'LineWidth', 1.5, 'EdgeColor', 'k')
end

% figure
% hold on
% keyList = keys(cellDict);
% for i = 1:length(keyList)
%     cellDict(keyList(i)).plotCell()
% end

% figure
% hold on
% for i = 1:length(keyList)
%     cellDict(keyList(i)).controlInvariantSet.projection([1,2]).plot()
% end

% figure
% cellDict("c_11").constraintSet.plot();
% hold on
% cellDict("c_11").controlInvariantSet.projection([1,2]).plot()


