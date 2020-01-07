function [A_aug, B_aug, C_aug, D_aug] = generateThmlPackModel(Temp, Current, model, dt)

global Param

Current = Current / (Param.num_parallel_cell*Param.num_strings);

A = [  -1/(Param.Rc*Param.Cc),  1/(Param.Rc*Param.Cc);
        1/(Param.Cs*Param.Rc), -1/(Param.Cs*Param.Rc) - 1/(Param.Cs*Param.Ru)];
    
B = [   2*Param.Re*Current/(Param.Cc*Param.num_parallel_cell * Param.num_strings);
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