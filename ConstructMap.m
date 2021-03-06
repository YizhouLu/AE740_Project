%% Construct the Map
TP_ll = [...
    42.279337, -83.747383;
    42.280463, -83.747316;
    42.280375, -83.744020;
    42.287009, -83.743580;
    42.289818, -83.738363;
    42.290861, -83.739468;
    42.302465, -83.735102;
    42.316201, -83.734143;
    42.316455, -83.727154;
    42.317337, -83.707657;
    42.311767, -83.707569;
    42.310812, -83.717121;
    42.309346, -83.717147;
    42.309297, -83.715551;
    42.305077, -83.707516;
    42.302616, -83.707208;
    42.302584, -83.704490;
    42.295370, -83.704930;
    42.295668, -83.707246;
    42.294086, -83.709680;
    42.293971, -83.712605;
    42.290393, -83.712707;
    42.290391, -83.717714];

TP_xy = zeros(2,length(TP_ll));
TP_xy(:,1) = [0;0];
PATH = [];
D = 0;
for i = 2:length(TP_ll)
    dlat = TP_ll(i,1) - TP_ll(1,1);
    dlong = TP_ll(i,2) - TP_ll(1,2);
    TP_xy(:,i) = [dlong*111.320*1000*cosd(TP_ll(i,1)); dlat*110.574*1000];
    
    distance = sqrt((TP_xy(1,i) - TP_xy(1,i-1))^2 + (TP_xy(2,i) - TP_xy(2,i-1))^2);
    direction = (TP_xy(:,i) - TP_xy(:,i-1))/distance;
    PATH = [PATH, TP_xy(:,i-1) + (0:0.05:distance).*direction];
    D = D+distance;
end

figure(1); hold on; grid on; axis equal
scatter(TP_xy(1,:),TP_xy(2,:))
plot(TP_xy(1,:),TP_xy(2,:))
scatter(PATH(1,:), PATH(2,:))

save('MAP', 'PATH')


