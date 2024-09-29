classdef MidLevelControl
    properties
        % Weights consist of cost matrices Q, R and Qf(for terminal cost)
        weight
        positionConstraints
        velocityConstraints
        stateConstraints
        terminalConstraints
        inputConstraints

        % combinedStateConstraints
        % combinedInputConstraints

        uc
        yc

        costmat

        counter
        terminate

    end

    methods
        % High level input consists of EnvCell of current and next partition
        function midControl = MidLevelControl(weight, velocityConstraints, inputConstraints, midSystem)
            midControl.weight = weight;
            % midControl = midControl.getConstraints(highLevelInput);
            midControl.velocityConstraints = velocityConstraints;
            midControl.inputConstraints = inputConstraints;
            midControl.costmat = costgen(midSystem.predmod, midControl.weight, midSystem.dim);
            midControl.counter = 0;
            midControl.terminate = true;

        end

        % High level input is a pair consisting of the current cell and the next cell as Polyhedron objects
        function midControl = getConstraints(midControl, highLevelInput)
            currentCell = highLevelInput{1};
            terminalCell = highLevelInput{2};
            P(1) = currentCell.constraintSet;
            P(2) = terminalCell.constraintSet;
            U = PolyUnion(P);
            midControl.positionConstraints = U.convexHull;
            midControl.terminalConstraints = terminalCell.controlInvariantSet;
            midControl.stateConstraints = Polyhedron(blkdiag(midControl.positionConstraints.A, midControl.velocityConstraints.A), [midControl.positionConstraints.b; midControl.velocityConstraints.b]);

        end

        function midControl = runMPC(midControl, midSystem)
            nx = midSystem.dim.nx;
            nu = midSystem.dim.nu;
            N = midSystem.dim.N;

            Q = midControl.weight.Q;
            R = midControl.weight.R;
            Qf = midControl.weight.Qf;

            x = sdpvar(nx, N+1);
            u = sdpvar(nu, N);
            midControl.uc = zeros(nu, N);
            midControl.yc = zeros(nx, N+1);

            cost = 0;

            x0 = midSystem.xc;
            xf = midSystem.xf;

            A = midSystem.sys.A;
            B = midSystem.sys.B;

            constraints = x(:,1) == x0;

            for k = 1:N 
                % disp(size(x(:,k)))
                % disp(xf)
                % disp(size(x(3:4,k)))
                % disp(size(Q))
                cost = cost + (x(:,k) - [xf; 0;0])' * Q * (x(:,k) - [xf; 0;0]) + u(:, k)' * R * u(:, k);
                % constraints = [constraints, ...
                %                x(:,k+1) == A*x(:,k) + B*u(:,k), ...
                %                midControl.positionConstraints.A * x(1:2,k) <= midControl.positionConstraints.b, ...
                %                midControl.velocityConstraints.A * x(3:4,k) <= midControl.velocityConstraints.b, ...
                %                midControl.inputConstraints.A * u(:,k) <= midControl.inputConstraints.b];
                
                               
                constraints = [constraints, ...
                               x(:,k+1) == A*x(:,k) + B*u(:,k), ...
                               midControl.inputConstraints.A * u(:,k) <= midControl.inputConstraints.b];
                if (N-k-midControl.counter) >= 0
                    constraints = [constraints, ...
                                   midControl.positionConstraints.A * x(1:2,k) <= midControl.positionConstraints.b, ...
                                   midControl.velocityConstraints.A * x(3:4,k) <= midControl.velocityConstraints.b];
                else 
                    constraints = [constraints, ...
                                   midControl.terminalConstraints.A * x(:,k) <= midControl.terminalConstraints.b];
                end
            end

            cost = cost + (x(:,N+1) - [xf; 0;0])' * Qf * (x(:,N+1) - [xf; 0;0]);
            constraints = [constraints, ...
                           midControl.terminalConstraints.A * x(:,N+1) <= midControl.terminalConstraints.b];
                        %    midControl.velocityConstraints.A * x(3:4,N+1) <= midControl.velocityConstraints.b];

            optimalControl = optimize(constraints, cost);

            if optimalControl.problem == 0
                midControl.uc = value(u);
                % disp(u(:,k))
                % midSystem.inputHistory = [midSystem.inputHistory, midSystem.uc(:,1)];

                midControl.yc = value(x);

                % midSystem.xc = A*midSystem.xc + B*midSystem.uc(:,1);
                % midSystem.stateHistory = [midSystem.stateHistory, midSystem.xc];

                % midControl.counter = midControl.counter + 1;
            else
                disp("Optimization problem failed to solve");
                fprintf("Robot:%s, counter:%d\n", midSystem.id, midControl.counter);
            end

            % terminalPositionConstraints = midControl.terminalConstraints.projection(1:2);
            % if terminalPositionConstraints.contains(midSystem.xc(1:2))
            %     midControl.terminate = true;
            %     midControl.counter = 0;
            % else
            %     midControl.counter = midControl.counter + 1;
            % end
        end


        function midControl = runMPC2(midControl, midSystem)
            dim = midSystem.dim;
        
            % weight = midSystem.weight; 
        
            midControl.uc = zeros(dim.nu * dim.N, 1);
            % midControl.yc = zeros(dim.nx * (dim.N+1), 1);
        
            x0 = midSystem.xc;
            xf = midSystem.xf;
        
            u = sdpvar(dim.nu * dim.N, 1);
        
            % tempmatState = blkdiag(midControl.positionConstraints.A, midControl.velocityConstraints.A);
            % combinedStateConstraints.A = [];
            % combinedInputConstraints.A = [];
            % disp(size(tempmatState))
            % disp(size(midControl.terminalConstraints.A))
            % for i = 1:dim.N
            %     % if i <= dim.N-midControl.counter
            %     combinedStateConstraints.A = blkdiag(combinedStateConstraints.A, tempmatState);
            %     % else 
            %         % combinedStateConstraints.A = blkdiag(combinedStateConstraints.A, tempmatState);
            %     % end
                
            %     % if i <= dim.N 
            %         % combinedInputConstraints.A = blkdiag(combinedInputConstraints.A, midControl.inputConstraints.A);
            %     % end
            % end
            % combinedStateConstraints.A = blkdiag(combinedStateConstraints.A, midControl.terminalConstraints.A);

            % This works
            if midControl.counter == dim.N
                combinedStateConstraints.A = kron(eye(dim.N+1), midControl.terminalConstraints.A);
                combinedStateConstraints.b = repmat(midControl.terminalConstraints.b, midControl.counter + 1, 1);
            else 
                modifiedStateConstraints.A = kron(eye(dim.N - midControl.counter), midControl.stateConstraints.A);
                modifiedStateConstraints.b = repmat(midControl.stateConstraints.b, dim.N - midControl.counter, 1);
                modifiedTerminalConstraints.A = kron(eye(midControl.counter + 1), midControl.terminalConstraints.A);
                modifiedTerminalConstraints.b = repmat(midControl.terminalConstraints.b, midControl.counter + 1, 1);
                combinedStateConstraints.A = blkdiag(modifiedStateConstraints.A, modifiedTerminalConstraints.A);
                combinedStateConstraints.b = [modifiedStateConstraints.b; modifiedTerminalConstraints.b];
            end

            
            % disp(tempmatState)
            % disp(combinedStateConstraints.A)
            % disp(midControl.positionConstraints.b)
            % disp(midControl.velocityConstraints.b)
            % disp(combinedStateConstraints.b(1:16))

            % combinedStateConstraints.A = blkdiag(kron(eye(dim.N), tempmatState), midControl.terminalConstraints.A);
            % combinedStateConstraints.b = [repmat([midControl.positionConstraints.b; midControl.velocityConstraints.b], dim.N, 1); midControl.terminalConstraints.b];

            % This works
            combinedInputConstraints.A = kron(eye(dim.N), midControl.inputConstraints.A);
            combinedInputConstraints.b = repmat(midControl.inputConstraints.b, dim.N, 1);
            
            combinedStateEquation = midSystem.predmod.T * x0 + midSystem.predmod.S * u;
            
            
            % disp(size(combinedStateConstraints.A))
            % disp(size(combinedStateConstraints.b))
            % disp(size(combinedStateEquation))
            % disp(size(combinedInputConstraints.A))
            % disp(size(combinedInputConstraints.b))

            % constraints = [midControl.combinedInputConstraints.A * u <= midControl.combinedInputConstraints.b, ...  
            %                midControl.combinedStateConstraints.A * combinedStateEquation <= midControl.combinedStateConstraints.b];
            
            % This works               
            constraints = [combinedInputConstraints.A * u <= combinedInputConstraints.b, ...  
                           combinedStateConstraints.A * combinedStateEquation <= combinedStateConstraints.b];
            % constraints = [];
            
            % disp(size(u))
            % disp(size(midControl.costmat.H))
            % disp(size(midControl.costmat.h))
            % disp(size([x0;0;0]))
            % disp(size([xf;0;0]))
            objective = 0.5 * u'*midControl.costmat.H*u + (midControl.costmat.h*[x0; [xf;0;0]; [0;0]])'*u;
            options = sdpsettings('verbose', 0);
            optimalControl = optimize(constraints, objective, options);
        
            if optimalControl.problem == 0
                midControl.uc = value(u);
            else 
                disp("Optimization problem failed to solve");
                fprintf("Robot:%s, counter:%d\n", midSystem.id, midControl.counter);
            end
        
            clear u           
        
        end

        
    end
end



            





