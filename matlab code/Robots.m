classdef Robots
    properties
        idList
        numRobots

        x0List

        robotList
        cellList 

    end

    methods
        function robots = Robots(idList, x0List, cellDict)
            robots.idList = idList;
            robots.numRobots = length(idList);

            robots.x0List = x0List;

            robots = robots.addRobots();
            robots.cellList = cellDict;

        end

        function robots = addRobots(robots)
            for i = 1:robots.numRobots
                id = robots.idList(i);
                x0 = robots.x0List(:, i);
                robots.robotList = [robots.robotList, Robot(id, x0)];
            end
        end

        % function robots = initialiseRobots(robots, sys, dim, T)
        %     for i = 1:robots.numRobots
        %         robots.robotList(i) = robots.robotList(i).initialiseMidLevelSystem(sys, dim, T);
        %     end
        % end

        function robots = initialiseMidLevelSystem(robots, sys, dim)
            for i = 1:robots.numRobots
                robots.robotList(i).midSystem = MidLevelSystem(robots.robotList(i).id, robots.robotList(i).x0, sys, dim);
            end
        end

        function robots = initialiseMidLevelControl(robots, weight, velocityConstraints, inputConstraints)
            for i = 1:robots.numRobots
                robots.robotList(i).midControl = MidLevelControl(weight, velocityConstraints, inputConstraints, robots.robotList(i).midSystem);
            end
        end

        function highLevelInput = findHighLevelInput(robots, sequencePair)
            currentCell = lookup(robots.cellList, sequencePair(1));
            terminalCell = lookup(robots.cellList, sequencePair(2));
            highLevelInput = {currentCell, terminalCell};
        
        end

    end
end
        
% function midControl = runMPC2(midControl, midSystem)
%     dim = midSystem.dim;

%     % weight = midSystem.weight; 

%     midControl.uc = zeros(dim.nu * dim.N, 1);
%     % midControl.yc = zeros(dim.nx * (dim.N+1), 1);

%     x0 = midSystem.xc;
%     xf = midSystem.xf;

%     u = sdpvar(dim.nu * dim.N, 1);

%     modifiedStateConstraints.A = repelem([midControl.positionConstraints.A; midControl.velocityConstraints.A], dim.N - midControl.counter);
%     modifiedStateConstraints.b = repelem([midControl.positionConstraints.b; midControl.velocityConstraints.b], dim.N - midControl.counter);
%     modifiedTerminalConstraints.A = repelem([midControl.terminalConstraints.A], midControl.counter + 1);
%     modifiedTerminalConstraints.b = repelem([midControl.terminalConstraints.b], midControl.counter + 1);
%     finalStateConstraints.A = [modifiedStateConstraints.A; modifiedTerminalConstraints.A];
%     finalStateConstraints.b = [modifiedStateConstraints.b; modifiedTerminalConstraints.b];
%     finalStateEquation = midSystem.predmod.T * x0 + midSystem.predmod.S * u;

%     constraints = [repelem(midControl.inputConstraints.A, dim.N) * u <= repelem(midControl.inputConstraints.b), ...
%                    finalStateConstraints.A * finalStateEquation <= finalStateConstraints.b];

%     objective = 0.5 * u'*midControl.costmat.H*u + (midControl.costmat.h*(x0-xf))'*u;
%     optimalControl = optimize(constraints, objective);

%     if optimalControl == 0
%         midControl.uc = value(u);
%     else 
%         disp("Optimization problem failed to solve");
%         fprintf("Robot:%s, counter:%d\n", midSystem.id, midControl.counter);
%     end

%     clear u


% end

        

