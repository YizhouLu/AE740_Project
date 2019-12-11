load('SAMmodel.mat');
Tc = 25;

OCV0 = model.OCV0;
OCVrel = model.OCVrel;
OCV_curve = OCV0 + Tc * OCVrel;
SOC_curve = model.SOC;

coe = polyfit(SOC_curve, OCV_curve, 15);

x = linspace(0,1,200);
y = polyval(coe,x);
scatter(x,y)