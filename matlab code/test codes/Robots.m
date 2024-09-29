classdef Robots
    properties
        IdList
        NumRobots
        t 

        x0List
        xcList

        RobotList
        CellList

    end

    methods
        function robots = Robots(idList, x0List, cellDict)
            robots.IdList = idList;
            robots.NumRobots = length(idList);
            robots.t = 0;
            robots.x0List = x0List;
            robots.xcList = x0List;

            robots.RobotList = [];
            robots.CellList = cellDict;
        end

        function robots = addRobots(robots, N, Q, R, Qf, velocityConstraints, inputConstraints)
            for i = 1:robots.NumRobots
                x0 = robots.x0List(:, i);
                currentCell = robots.findCurrentCell(x0);
                robots.RobotList = [robots.RobotList, Robot(x0, N, Q, R, Qf, currentCell, velocityConstraints, inputConstraints)];
            end
        end

        function currentCell = findCurrentCell(robots, x0)
            currentCell = "None";
            k = keys(robots.CellList);
            v = values(robots.CellList);
            for i = 1:robots.NumRobots
                if v(i).contains(x0)
                    currentCell = k(i);
                    break
                end
            end
        end

        % LTL motion plan
        function robots = motionPlan(robots)
            for i = 1:robots.NumRobots
                robots.RobotList(i).TransitionSequence = ["c_11", "c_12", "c_22"];
            end
        end

        function [currentCell, nextCell] = getNextTransition(robots, i)
            currentCell = robots.RobotList(i).TransitionSequence(robots.RobotList(i).SequenceCounter);
            robots.RobotList(i).SequenceCounter = robots.RobotList(i).SequenceCounter + 1;
            if robots.RobotList(i).SequenceCompletion
                nextCell = "None";
            else
                if robots.RobotList(i).SequenceCounter >= length(robots.RobotList(i).TransitionSequence)
                    nextCell = "None";
                    robots.RobotList.SequenceCompletion = true;
                else
                    nextCell = robots.RobotList(i).TransitionSequence(robots.RobotList(i).SequenceCounter);
                end
            end
        end

        function robots = trajectoryPlan(robots)

        end
    end
end
