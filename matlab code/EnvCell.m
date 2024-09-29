classdef EnvCell
    properties
        state
        constraintSet
        controlInvariantSet
        sys
    end

    methods
        function cellObj = EnvCell(i, j, xmin, xmax, ymin, ymax, sys, constraints)
            % i - row index of the cell
            % j - column index of the cell
            % n - dimension of the cell space
            
            cellObj.state = sprintf('c_%d%d', i, j);
            H = [eye(2); -eye(2)];
            h = [xmax; ymax; -xmin; -ymin];
            cellObj.constraintSet = Polyhedron(H, h);

            cellObj = cellObj.findControlInvariantSet(sys, constraints);
            
        end

        function plotCell(partition)
            partition.constraintSet.plot(color='blue', alpha=0.1, linestyle='--')
            hold on
            center = [(partition.constraintSet.b(3) + partition.constraintSet.b(1))/2,...
                      (partition.constraintSet.b(4) + partition.constraintSet.b(2))/2];
            text(center(1), center(2), partition.state, 'HorizontalAlignment', 'center');
        end

        function cellObj = findControlInvariantSet(cellObj, sys, constraints)
            if constraints.delta == 0
                H = blkdiag(cellObj.constraintSet.A, constraints.velocity.A);
                h = [cellObj.constraintSet.b; constraints.velocity.b];
                stateConstraint = Polyhedron(H, h);
                cellObj.controlInvariantSet = sys.invariantSet('X', stateConstraint, 'U', constraints.input);
            elseif constraints.delta > 0
                d = constraints.delta;
                Hd = [eye(2); -eye(2)];
                hd = [d; d; d; d];
                Bd = Polyhedron(Hd, hd);
                modifiedConstraintSet = cellObj.constraintSet - Bd;
                % figure
                % modifiedConstraintSet.plot()

                H = blkdiag(modifiedConstraintSet.A, constraints.velocity.A);
                h = [modifiedConstraintSet.b; constraints.velocity.b];
                stateConstraint = Polyhedron(H, h);
                cellObj.controlInvariantSet = sys.invariantSet('X', stateConstraint, 'U', constraints.input);
            end
        end
    end
end
