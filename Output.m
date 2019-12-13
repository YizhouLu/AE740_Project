function [y, OCV] = Output(x, u, model, coe)
z  = x(3);
Vc = x(4);
e  = x(7);
Tc = x(10);
  
OCV = polyval(coe, z);
R0  = getParamESC('R0Param',Tc,model); % Ohmic resistance

Vt = OCV - R0*u - Vc;

y = [Vt;e;u;z;Tc];
end