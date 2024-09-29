classdef Robot
    properties
        id 
        x0
        xc
        xf
        uc
        yc
        N

        % Class objects of type EnvCell
        CurrentCell
        TerminalCell
        StateVelocityConstraints
        InputConstraints

        TerminalFlag

        Q
        R
        Qf

        TransitionSequence
        SequenceCounter
        SequenceCompletion

        StateHistory
        InputHistory
    end

    methods
        function robot = Robot(x0, N, Q, R, Qf, currentCell, velocityConstraints, inputConstraints)
            robot.id = cellObj.State;
            robot.x0 = x0;
            robot.xc = xc;
            robot.xf = zeros(4,1);
            robot.uc = zeros(2,N);
            robot.yc = zeros(4,N+1);
            robot.N = N;
            
            robot.Q = Q;
            robot.R = R;
            robot.Qf = Qf;

            robot.CurrentCell = currentCell;
            robot.TerminalCell = currentCell;
            robot.StateVelocityConstraints = velocityConstraints;
            robot.InputConstraints = inputConstraints;

            robot.TransitionSequence = [];
            robot.SequenceCounter = 0;
            robot.SequenceCompletion = false;

            robot.StateHistory = [];
            robot.InputHistory = [];
        end

        function robot = initMPC(robot, xf, terminalCell)
            robot.x0 = robot.xc;
            robot.xf = xf;
            robot.TerminalCell = terminalCell;
        end

        function robot = runMPC(robot, N, T)
            x = sdpvar(4, N+1);
            u = sdpvar(2, N);

            A = [eye(2) T*eye(2);
                 zeros(2) eye(2)];
            B = [zeros(2); eye(2)];

            cost = 0;

            P(1) = robot.CurrentCell.ConstraintSet;
            P(2) = robot.TerminalCell.ConstraintSet;
            U = PolyUnion(P);
            stateConstraints = U.convexHull;
            terminalConstraints = robot.TerminalCell.ControlInvariantSet;
            
            
            for k = 1:N
                cost = cost + (x(:,k) - [robot.xf; x(3:4,k)])' * robot.Q * (x(:,k) - [robot.xf; x(3:4,k)]) + u(:, k)' * robot.R * u(:, k);

                constraints = [x(:,1) == robot.x0, ...
                               x(:,k+1) == A*x(:,k) + B*u(:,k), ...
                               stateConstraints.A * x(1:2,k) <= stateConstraints.b, ...
                               robot.StateVelocityConstraints.A * x(3:4,k) <= robot.StateVelocityConstraints.b, ... 
                               robot.InputConstraints.A * u(:,k) <= robot.InputConstraints.b];
            
            end

            cost = cost + (x(:,N+1) - [robot.xf; x(3:4,N+1)])' * robot.Q * (x(:,N+1) - [robot.xf; x(3:4,N+1)]);
            constraints = [constraints, terminalConstraints.A * x(1:2,N+1) <= terminalConstraints.b, ...
                           robot.StateVelocityConstraints.A * x(3:4,N+1) <= robot.StateVelocityConstraints.b];

            % options = sdpsettings('solver', 'quadprog', 'verbose', 0);
            optimalControl = optimize(constraints, cost);

            if optimalControl.problem == 0
                robot.uc = value(u);
                robot.InputHistory = [robot.InputHistory, robot.uc(:,1)];

                robot.yc = value(x);

                robot.xc = A*robot.xc + B*robot.uc(:,1);
                robot.StateHistory = [robot.StateHistory, robot.xc];
            else 
                disp("Optimization problem failed to solve");
            end

            if norm(robot.xc(1:2) - robot.xf(1:2)) < 7.5
                robot.TerminalFlag = true;
            end 
        end           
    end
end


            


