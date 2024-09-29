classdef TopLevelSystem
    properties
        motionPlan
        sequenceCounter
        terminate
    end

    methods
        function topSystem = TopLevelSystem(motionPlan)
            topSystem.motionPlan = motionPlan;
            topSystem.sequenceCounter = 1;
            topSystem.terminate = false;
        end

        function [sequencePair, topSystem] = getSequencePair(topSystem)
            
            if topSystem.sequenceCounter + 1 > length(topSystem.motionPlan)
                topSystem.terminate = true;
                sequencePair = [];
            else 
                sequencePair = [topSystem.motionPlan(topSystem.sequenceCounter), topSystem.motionPlan(topSystem.sequenceCounter + 1)];
                topSystem.sequenceCounter = topSystem.sequenceCounter + 1;
            end       
            % sequencePair = [topSystem.motionPlan(topSystem.sequenceCounter), topSystem.motionPlan(topSystem.sequenceCounter + 1)];
            % topSystem.sequenceCounter = topSystem.sequenceCounter + 1;
            
            % if topSystem.sequenceCounter + 1 > length(topSystem.motionPlan)
            %     topSystem.terminate = true;
            % end
        
        end

    end
end