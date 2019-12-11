function [H, L, G, W, T] = formQPMatrices_KronMethod(sys, N, penalty, limit)
A = sys.A;
B = sys.B; 
C = sys.C; 
D = sys.D;

Np = N.prediction;
Nc = N.control;

Q = penalty.Q;
R = penalty.R;

num_state = 11;
num_input = 1;
num_output = 5;

Phi   = zeros((Np+1)*num_output, num_state);
Gamma = zeros((Np+1)*num_output, Nc*num_input);

for i = 0:Np
    Phi(i*num_output+(1:num_output), :) = C*A^i;
    if i == 0
        Gamma(1:num_output, 1:num_input) = D;
    else
        Gamma(i*num_output+(1:num_output), :) = [C*A^(i-1)*B, Gamma((i-1)*num_output+(1:num_output), 1:end-num_input)];
    end
end

Phi   = Phi(num_output+1:end,:);
Gamma = Gamma(num_output+1:end,:);

Qbar = kron(eye(Np), Q);
Rbar = kron(eye(Nc), R); 

H = 2.*(Gamma'*Qbar*Gamma + Rbar);
L = 2.*(Gamma'*Qbar*Phi);

G = [...
     eye(Nc); 
    -eye(Nc); 
     Gamma;
    -Gamma];

DeltaUmax = kron(ones(Nc, 1), limit.du.max);
DeltaUmin = kron(ones(Nc, 1), limit.du.min);
Ymax      = kron(ones(Np, 1), limit.y.max);
Ymin      = kron(ones(Np, 1), limit.y.min);

W = [...
     DeltaUmax; 
    -DeltaUmin; 
     Ymax; 
    -Ymin];

T = [...
     zeros(Nc,num_state); 
     zeros(Nc,num_state); 
    -Phi; 
     Phi];

end
