classdef Bus < handle
    properties
        Position_idx
        x
        y
        SOC   % 0: depletion,   100; full
        State % 0: running,     1: charging
        graph_handle
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
                if obj.Position_idx > length(MAP)
                    obj.Position_idx = length(MAP);
                end
                obj.x = MAP(1, obj.Position_idx);
                obj.y = MAP(2, obj.Position_idx);
            else                % if the bus is charging
                obj.x = MAP(1, obj.Position_idx);
                obj.y = MAP(2, obj.Position_idx);
            end
        end
        
        function updateSOC(obj) % TODO: REPLACE THIS FUNCTION BY SIMULINK
            if obj.State == 0   % if the bus is running
                obj.SOC = obj.SOC - 0.5;
            else                % if the bus is charging        
                obj.SOC = obj.SOC + 2;
            end            
        end
        
        function updateState(obj, BusStop_idx)
            if sum(obj.Position_idx == BusStop_idx) && obj.SOC < 80
                obj.State = 1;
            else
                obj.State = 0;
            end
        end
        
        function graph(obj)
            if obj.State == 0
                obj.graph_handle = scatter(obj.x,obj.y,'rd','filled');
            else
                obj.graph_handle = scatter(obj.x,obj.y,'gd','filled');
            end
        end
        
    end
end