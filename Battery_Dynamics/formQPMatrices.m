function [H,L,G,W,T,IMPC] = formQPMatrices(A, B, C, D, Np, Nc, r, ylimit, ulimit)

% Inputs: A, B, C, D are the original state space matrices of the system
% dynamics. X0 is the set of initial conditions for the EXTENDED state
% vector which accounts for the direct feedthrough matrix D. Np is the
% prediction horizon. Nc is the control horizon. Np >= Nc. r is the
% weighting for the DeltaU in the cost function -> must be nonzero so that
% we have a positive definite matrix I think.. SP is the set point we wish
% to track (ie. set point for the State of Charge).

% Outputs: H is the Hessian. q is the linear term of the cost function. G
% is the part of the constraint matrix multiplying DeltaU. W is the matrix
% of stacked Xmax, Xmin, Ymax and Ymin constraints. T is the affine portion
% of the constraint which is multiplied by X0.

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % First form the modified A, B and C matrices to account for
% % the direct feedthrough term arising from the D matrix. Tried to
% % generalize for arbitrary input case. Time will tell if it worked lol.
% 
% A = [A, B; zeros(size(B,2),size(A,2)), eye(size(B,2))];
% B = [zeros(size(B,1),1); ones(size(B,2),1)];
% C = [C, D];
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % X = M*x0 + S*DeltaU
% 
% M = [];
% for i = 1:Np
%     M = [M;A^i];
% end
% 
% col_S = [];
% for i = 1:Np
%     col_S = [col_S;(A^(i - 1)*B)];
% end
% 
% S = zeros(Np*size(A,1),Nc*size(B,2));
% 
% for i = 1:Nc
%     if i == 1
%         S = col_S;
%     else
%         col_S = [zeros(size(B,1),1);col_S]; 
%         col_S((end - size(B,1)) + 1:end,:) = [];
%         S = [S,col_S];
%     end
% end

%%%%%%%%%%%%%%%
% Y = Phi*A*x0 + Gamma*DeltaU

Phi = [];
for i = 1:(Np + 1)
    if i == 1
        Phi = C;
    else
        Phi = [Phi;C*A^(i-1)];
    end
end

% % we actually need Phi*A in the new formulation so spit that out of
% % the function instead of just Phi
% 
% Phi_mult_A = Phi*A;

col_Gamma = [];
for i = 1:(Np + 1)
    if i == 1
        col_Gamma = D;
    else
        col_Gamma = [col_Gamma;C*(A^(i - 2))*B];
    end
end

Gamma = zeros(size(C,1)*(Np + 1),(Nc*size(B,2)));
for i = 1:Nc
    if i == 1
        Gamma = col_Gamma;
    else
        col_Gamma = [zeros(size(C,1),1);col_Gamma]; 
        col_Gamma((end - size(C,1)) + 1:end,:) = [];
        Gamma = [Gamma,col_Gamma];
    end
end

% Q bar is a symmetric & positive-semi definite matrix. We wish to penalize
% the error between measured SOC and SOC set point. The output vector is 
% [e, Vt, i, z]. Here the error is plucked out by Q so that it is
% penalized, while Vt, i and z are all constrained outputs.
Q = diag([1, 0, 0, 0]); 
Qbar = blkdiag(Q);

while size(Qbar) ~= (Np + 1)*size(C,1)
    Qbar = blkdiag(Q,Qbar);
end

% Rbar is a symmetric & positive definite matrix. It penalizes the the
% control increment by value r.
Rbar = r*eye(Nc);

%%%%%%%%%%%%%%%
% Cost Function

H = Gamma'*Qbar*Gamma + Rbar;
L = Gamma'*Qbar*Phi;

% SP = SP*ones(size(Gamma,1),1);
% H = 2.*(Gamma'*Qbar*Gamma + r);
% q = 2.*Gamma'*Qbar*(Phi_mult_A*X0 - SP);
% % we might need to move X0 outside the function since that's what we did
% % for HW3 P2, but here we have two terms so we might have to output both if
% % we go that route


%%%%%%%%%%%%%%%
% Handle constraints

% should the DeltaU constrain be Np x Nc or just Nc x Nc? If the control
% horizon is 3 and prediction horizon is 10, DeltaU will be constant for
% steps 4, 5, 6, 7, 8, 9, 10 so I think we only have to constrain the first
% three.. not sure though.
G = [eye(Nc); -eye(Nc); Gamma; -Gamma];

DeltaUmax = [];
DeltaUmin = [];
Ymax = [];
Ymin = [];

for i = 1:Nc
    DeltaUmax = [DeltaUmax;ulimit.max];
    DeltaUmin = [DeltaUmin;ulimit.min];
end 

while size(Ymin,1) ~= size(Gamma,1)
    Ymax = [Ymax;ylimit.max];
    Ymin = [Ymin;ylimit.min];
end 

W = [DeltaUmax; -DeltaUmin; Ymax; -Ymin];
T = [zeros(Nc,size(A,1)); zeros(Nc,size(A,1)); 
    -Phi; Phi];

IMPC = [1, zeros(1,(Nc-1))];

end
