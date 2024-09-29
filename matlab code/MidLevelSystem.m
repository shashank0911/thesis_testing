classdef MidLevelSystem
    properties
        % sys consists of system matrices A and B
        sys
        id
        t

        x0
        xc
        xf
        uc
        yc

        % Dim consists of number of states(nx), inputs(nu) and prediction horizon(N) 
        dim
        predmod

        stateHistory
        inputHistory

        midControlInit

    end

    methods
        function midSystem = MidLevelSystem(id, x0, sys, dim)
            % midSystem.sys.A = [eye(dim.nx), T*eye(dim.nx); zeros(dim.nx) eye(dim.nx)];
            % midSystem.sys.B = [zeros(dim.nu); eye(dim.nu)];
            midSystem.sys = sys;
            midSystem.id = id;
            midSystem.t = 1;
            midSystem.x0 = x0;
            midSystem.xc = x0;
            midSystem.xf = midSystem.x0(1:2,:);
            midSystem.uc = zeros(dim.nu * dim.N, 1);
            % midSystem.yc = zeros(dim.nx * (dim.N+1), 1);
            midSystem.dim = dim;
            midSystem.predmod = predmodgen(midSystem.sys, midSystem.dim);
            midSystem.stateHistory = zeros(dim.nx, dim.simsteps+1);
            midSystem.inputHistory = zeros(dim.nu, dim.simsteps);
            midSystem.stateHistory(:, 1) = x0;
            midSystem.midControlInit = false;
        
        end

        function [midSystem, midControl] = getMPC(midSystem, midControl)
            midControl = midControl.runMPC2(midSystem);
            midSystem.uc = midControl.uc;
            % midSystem.yc = midControl.yc;

            midSystem.xc = midSystem.sys.A * midSystem.xc + midSystem.sys.B * midSystem.uc(1:midSystem.dim.nu, 1);

            terminalPositionConstraints = midControl.terminalConstraints.projection(1:2);

            if terminalPositionConstraints.contains(midSystem.xc(1:2))
                fprintf("We are inside!\t")
                fprintf("Robot:%s, counter:%d\n", midSystem.id, midControl.counter);
                midControl.terminate = true;
                midControl.counter = 0;
            else
                if midControl.counter == midSystem.dim.N
                    midControl.counter = 0;
                    fprintf("Target cell not reached in N steps by robot %s", midSystem.id);
                else
                    midControl.counter = midControl.counter + 1;
                end
            end

            midSystem.inputHistory(:, midSystem.t) = midSystem.uc(1:midSystem.dim.nu);
            midSystem.stateHistory(:, midSystem.t+1) = midSystem.xc;
        end

        function F_2(midSystem)

        end

        function F_1Inv(midSystem)

        end
    end
end

        