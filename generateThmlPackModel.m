function [A_aug, B_aug, C_aug, D_aug] = generateThmlPackModel(Temp, Current, model, dt)

Cc = 80;   % J/K  (tune)
Cs = 10;   % J/K  (tune)
Re = 1;    % Ohms (combo of R1 and R0)
Rc = 1;    % K/W  (tune)
Ru = 4;    % K/W  (tune)

num_parallel_cell = 20;
num_strings       = 14;

Current = Current / (num_parallel_cell*num_strings);

A = [  -1/(Rc*Cc),  1/(Rc*Cc);
        1/(Cs*Rc), -1/(Cs*Rc) - 1/(Cs*Ru)];
    
B = [   2*Re*Current/Cc;
                      0];
    
C = zeros(1,2);

D = 0;

sys_c = ss(A,B,C,D);
sys_d = c2d(sys_c, dt);

A_aug = [sys_d.A, zeros(2); eye(2), eye(2)];
B_aug = [sys_d.B; zeros(2,1)];
C_aug = [1 0 1 0];
D_aug = 0;

end