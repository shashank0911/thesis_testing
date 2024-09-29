classdef Robot
    properties
        id
        x0

        topSystem
        midSystem
        midControl
        lowSystem 
        lowControl
    
    end
    
    methods
        function robot = Robot(id, x0)
            robot.id = id;
            robot.x0 = x0;

        end

        % function robot = initialiseMidLevelSystem(robot, sys, dim, T)
        %     robot.midSystem = MidLevelSystem(robot.x0, sys, dim, T);
        % end

        % function robot = initialiseMidLevelControl(robot, weight, highLevelInput, velocityConstraints, inputConstraints)
        %     robot.midControl = MidLevelControl(weight, highLevelInput, velocityConstraints, inputConstraints);
        % end

    end
end
