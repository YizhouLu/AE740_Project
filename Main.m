load('MAP.mat')

plot(PATH(1,:),PATH(2,:));axis equal;grid on;hold on

BusStop_idx = round(linspace(1,1229,20));
for j = 1:length(BusStop_idx)
    scatter(PATH(1,BusStop_idx(j)),PATH(2,BusStop_idx(j)),'r*')
end

%% 
Init.Position_idx = 1;
Init.SOC = 100;
Init.State = 0;

Bus1 = Bus(Init);
Bus2 = Bus(Init);
for i = 1:1:1000
    Bus1.updatePosition(PATH);
    Bus1.updateSOC();
    
    if i >= 50
        Bus2.updatePosition(PATH);
        Bus2.updateSOC();
    end
    
    if Bus1.State == 0
        Bus1_symbol = scatter(Bus1.x, Bus1.y, 'rd');
    else
        Bus1_symbol = scatter(Bus1.x, Bus1.y, 'gd');
    end
    
    if Bus2.State == 0
        Bus2_symbol = scatter(Bus2.x, Bus2.y, 'rd');
    else
        Bus2_symbol = scatter(Bus2.x, Bus2.y, 'gd');
    end
    
    
    
    
    pause(0.1);    
    delete(Bus1_symbol);
    delete(Bus2_symbol);
end

