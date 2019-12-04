clc; clear; close all

R1 = 0.0128;
C1 = 168.2566;
R0 = 0.1034;
Q = 2.1605;
dt = 0.001;
alpha0 = 3.613;
alpha1 = 0.4631;


A = [   1,                0, 0;
        0, exp(-dt/(R1*C1)), 0;
        0,                0, 1];
    
B = [                   -dt/Q;
      R1*(1-exp(-dt/(R1*C1)));
                            0];
  
C = [alpha1, -1, alpha0;
          0,  0,      1];
  
D = [-R0;0];

Aaug = [A,B;zeros(1,3),1];
Baug = [zeros(3,1);1];
Caug = [C,D];

x = zeros(4, 50);
x(:,1) = [0.2; 0; 1; -1];
y = zeros(2, 50);

for i = 1:50
   x(:,i+1) = Aaug*x(:,i) + Baug*(0);
   y(:,i)   = Caug*x(:,i);
end

plot(x(1,:))
