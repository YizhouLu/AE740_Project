function [Phi_mult_Atilde,Gamma,M,S] = formQPMatrices(A, B, C, D, Np, Nc)

% This function forms the slightly modified state transition matrices, but
% for both X = M*x0 + S*DeltaU and Y = Phi*Atilde*x0 + Gamma*DeltaU.

% First form the modified Atilde, Btilde and Ctilde matrices to account for
% the direct feedthrough term arising from the D matrix.

Atilde = [A, B; zeros(1,size(A,2)), eye(1)];
Btilde = [zeros(size(B,1),1); eye(1)];
Ctilde = [C, D];


% Form the components of X (ie. M and S)

M = [];
for i = 1:(Np + 1)
    M = [M;Atilde^i];
end

col_S = [];
for i = 1:(Np + 1)
    col_S = [col_S;(Atilde^(i - 1)*Btilde)];
end

S = zeros(size(Atilde,1)*(Np + 1),(Nc*size(Btilde,2)));

for i = 1:(Nc + 1)
    if i == 1
        S = col_S;
    else
        col_S = [zeros(size(Btilde,1),1);col_S]; 
        col_S((end - size(Btilde,1)) + 1:end,:) = [];
        S = [S,col_S];
    end
end


% Form the components of Y (ie. Phi and Gamma)

Phi = [];
for i = 1:(Np + 1)
    Phi = [Phi;Ctilde*Atilde^(i-1)];
end

% we actually need Phi*Atilde in the new formulation so spit that out of
% the function instead of just Phi

Phi_mult_Atilde = Phi*Atilde;

col_Gamma = [];
for i = 1:(Np + 1)
    col_Gamma = [col_Gamma;Ctilde*(Atilde^(i - 1))*Btilde];
end

Gamma = zeros(size(Ctilde,1)*(Np + 1),(Nc*size(Btilde,2)));
for i = 1:(Nc + 1)
    if i == 1
        Gamma = col_Gamma;
    else
        col_Gamma = [zeros(size(Ctilde,1),1);col_Gamma]; 
        col_Gamma((end - size(Ctilde,1)) + 1:end,:) = [];
        Gamma = [Gamma,col_Gamma];
    end
end









% while size(Qbar) ~= N*size(A)
%     Qbar = blkdiag(Q,Qbar); % 5N x 5N (symmetric & positive-semi definite)
% end
% 
% Rbar = eye(N);              % N x N (symmetric and positive definite)  

%%%%%%%%%%%%%%%%


% cost function transformation



% H = Gamma'*Qbar*Gamma + Rbar;
% L = Gamma'*Qbar*M;


% Ignore constraints for now

% % Handle constraints
% 
% G = [S; -S; eye(N); -eye(N)];
% 
% Xmax = [];
% Xmin = [];
% 
% while size(Xmin,1) ~= size(S,1)
%     Xmax = [Xmax;xlimit.max'];
%     Xmin = [Xmin;xlimit.min'];  
% end
% 
% dUmax = [];
% dUmin = [];
% for i = 1:N
%     dUmax = [dUmax;ulim.max];
%     dUmin = [dUmin;ulim.min];
% end 
% 
% W = [Xmax; -Xmin; dUmax; -dUmin];
% T = [-M; M; zeros(N,size(A,1)); zeros(N,size(A,1))];
% 
% IMPC = [1, zeros(1,(N-1))];

end
