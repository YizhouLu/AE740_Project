classdef Bus < handle
    properties
        Distance     % overall distance on the PATH 
        Position_idx % current node on the PATH
        direction    % 0: go forward   1: go back 
        vel
        isRunning    % 1: running,     0: charging
        x            % only for updating graph
        y            % only for updating graph
        graph_handle % only for updating graph
        dz
        dVc
        SOC          % 0: depletion,   1; full
        Vc
        i_last
        e
        dTc
        dTs
        Tc
        Ts
        X
        
    end
    
    methods
        function obj = Bus(init)
            obj.Distance        = init.Distance;    % update when the bus is running
            obj.Position_idx    = init.Position_idx;% update when the bus is running
            obj.direction       = init.direction;
            obj.isRunning       = init.isRunning;   % update every iteration
            obj.dz      = init.dz;                  % update when the bus is charging
            obj.dVc     = init.dVc;                 % update when the bus is charging
            obj.SOC     = init.SOC;                 % update every iteration
            obj.Vc      = init.Vc;                  % update when the bus is charging
            obj.i_last  = init.i_last;              % update when the bus is charging
            obj.e       = init.e;                   % update when the bus is charging
            obj.dTc     = init.dTc;                 % update when the bus is charging
            obj.dTs     = init.dTs;                 % update when the bus is charging
            obj.Tc      = init.Tc;                  % update when the bus is charging (should be update every iteration)
            obj.Ts      = init.Ts;                  % update when the bus is charging (should be update every iteration)
            obj.X      = [obj.dz; obj.dVc; obj.SOC; obj.Vc; 1; obj.i_last; obj.e; obj.dTc; obj.dTs; obj.Tc; obj.Ts];
        end
        
        function updatePosition(obj, MAP, VELOCITY, iter)
            if obj.isRunning
                index = rem(iter,599);                  % keep iterating through VELOCITY
                if index == 0
                    index = 599;
                end
                obj.vel = VELOCITY(index, 2);               % get current velocity at the present node 
                if obj.direction == 0
                    obj.Distance = obj.Distance + obj.vel * 1;  % get current distance traveled by the bus at the present node
                else 
                    obj.Distance = obj.Distance - obj.vel * 1;
                end
                if obj.Distance == 0
                    obj.Position_idx = 1;
                else
                    obj.Position_idx = round(obj.Distance/0.05);
                    if obj.Position_idx > length(MAP)
                        obj.Position_idx = length(MAP);
                    elseif obj.Position_idx < 1
                        obj.Position_idx = 1;      
                    end
                end
            end
            obj.x = MAP(1, obj.Position_idx);
            obj.y = MAP(2, obj.Position_idx);
        end
        
        function updateSOC(obj, SOC)
            
            obj.SOC = SOC;
%             if obj.isRunning
%                 if obj.vel > 0.03
%                     obj.SOC = SOC;
%                 end
%                 obj.SOC
%             else
%                 obj.SOC
%             end            
        end
        
        
        function updateState(obj, BusStop_idx)    
            global Param
            if sum(obj.Position_idx == BusStop_idx) && obj.SOC < Param.SOC_setpoint
                obj.dz      = obj.X(1);
                obj.dVc     = obj.X(2);
                obj.SOC     = obj.X(3);
                obj.Vc      = obj.X(4);
                obj.i_last  = obj.X(6);
                obj.e       = obj.X(7);
                obj.dTc     = obj.X(8);
                obj.dTs     = obj.X(9); 
                obj.Tc      = obj.X(10); 
                obj.Ts      = obj.X(11);
                obj.isRunning = 0;
                if obj.direction == 0
                    obj.direction = 1;
                else
                    obj.direction = 0;
                end
            else
                obj.X = [obj.dz; obj.dVc; obj.SOC; obj.Vc; 1; obj.i_last; obj.e; obj.dTc; obj.dTs; obj.Tc; obj.Ts];
                obj.isRunning = 1;
            end
        end
        
        function graph(obj, iter, figure_path, figure_bus1_SOC)
            subplot(1, 2, figure_path);
            if obj.isRunning
                obj.graph_handle = scatter(obj.x,obj.y,'rd','filled');
            else
                obj.graph_handle = scatter(obj.x,obj.y,'gd','filled');
            end
            subplot(1, 2, figure_bus1_SOC)
            scatter(iter, obj.SOC, 'b');
        end
        
    end
end