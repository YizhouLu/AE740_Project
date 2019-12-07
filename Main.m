close all
load('MAP.mat')

plot(PATH(1,:),PATH(2,:),'k','linewidth',2);axis equal;grid on;hold on

BusStop_idx = round(linspace(1,1229,20));
for j = 1:length(BusStop_idx)
    scatter(PATH(1,BusStop_idx(j)),PATH(2,BusStop_idx(j)),'b','filled')
end

%% 
Init.Position_idx = 1;
Init.SOC = 100;
Init.State = 0;

Bus1 = Bus(Init);
Bus2 = Bus(Init);
Bus3 = Bus(Init);
tic;
t_last = 0;
while true
    if toc > 0
        Bus1.updatePosition(PATH);
        Bus1.updateSOC();
        Bus1.updateState(BusStop_idx);      
    end
    
    if toc > 15 
        Bus2.updatePosition(PATH);
        Bus2.updateSOC();
        Bus2.updateState(BusStop_idx);
    end
    
    if toc > 30 
        Bus3.updatePosition(PATH);
        Bus3.updateSOC();
        Bus3.updateState(BusStop_idx);
    end
    
    Bus1.graph();
    Bus2.graph();
    Bus3.graph();
    
    while toc - t_last < 0.1
        drawnow;
    end
    t_last = toc;
    
    delete(Bus1.graph_handle);
    delete(Bus2.graph_handle);
    delete(Bus3.graph_handle);
end

