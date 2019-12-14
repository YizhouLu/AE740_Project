load('SAMmodel.mat');

Tc = 25;

OCV0 = model.OCV0;
OCVrel = model.OCVrel;

% OCV_curve = OCV0 + Tc * OCVrel;

num_series        = 96;
num_parallel_cell = 20;
num_strings       = 14;

OCV0_pack   = num_series.*OCV0;
OCVrel_pack = num_series.*OCVrel;

OCV_curve = OCV0_pack + Tc * OCVrel_pack;

SOC_curve = model.SOC;

coe = polyfit(SOC_curve, OCV_curve, 15);

x = linspace(0,1,200);
y = polyval(coe,x);
scatter(x,y)
hold on; grid on
plot(SOC_curve, OCV_curve)

save('SOC_OCV_pack_coe.mat','coe');