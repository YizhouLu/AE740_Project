classdef Bus < handle
    properties
        Position_idx
        x
        y
        SOC   % 0: depletion,   100; full
        State % 0: running,     1: charging
    end
    methods
        function obj = Bus(init)
            obj.Position_idx = init.Position_idx;
            obj.SOC = init.SOC;
            obj.State = init.State;
        end
        
        function updatePosition(obj, MAP)
            if obj.State == 0   % if the bus is running
                obj.Position_idx = obj.Position_idx + 1;
                obj.x = MAP(1,obj.Position_idx);
                obj.y = MAP(2,obj.Position_idx);
                if obj.Position_idx == 66 || obj.Position_idx == 130
                    obj.State = 1;
                    
                end
            else                % if the bus is charging
                obj.x = MAP(1,obj.Position_idx);
                obj.y = MAP(2,obj.Position_idx);
            end
        end
        
        function updateSOC(obj) % TODO: REPLACE THIS FUNCTION BY SIMULINK
            if obj.State == 0   % if the bus is running
                obj.SOC = obj.SOC - 0.5;
            else                % if the bus is charging        
                if obj.SOC >= 80% if the bus is fully charged
                    obj.State = 0;
                else            % if the bus needs to be charged
                    obj.SOC = obj.SOC + 2;
                end
            end            
        end
        
    end
end