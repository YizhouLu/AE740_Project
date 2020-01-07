function [A_aug, B_aug, C_aug, D_aug] = generateThmlModel(Temp, Current, model, dt)

Cc = 80;   % J/K  (tune)
Cs = 10;   % J/K  (tune)
Re = 2;    % Ohms (combo of R1 and R0)
Rc = 1;    % K/W  (tune)
Ru = 4;    % K/W  (tune)

A = [  -1/(Rc*Cc),  1/(Rc*Cc);
        1/(Cs*Rc), -1/(Cs*Rc) - 1/(Cs*Ru)];
    
B = [   2*Re*Current/Cc;
                     0];
                 
C = zeros(1,2);

D = 0;

sys_c = ss(A,B,C,D);
sys_d = c2d(sys_c, dt);

% augment the thermal model so that it can be concatenated with the
% electrical model in the for loop where MPC runs

A_aug = [sys_d.A, zeros(2); eye(2), eye(2)];
B_aug = [sys_d.B; zeros(2,1)];
C_aug = [1 0 1 0];
D_aug = 0;

end