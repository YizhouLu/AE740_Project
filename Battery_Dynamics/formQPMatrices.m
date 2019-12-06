function [H,q,G,W,T,Gamma,SP,Phi_mult_Atilde,IMPC,Atilde,Btilde,Ctilde] = ...
formQPMatrices(A, B, C, D, X0, Np, Nc, r, SP, xlimit, ylimit, ulimit)

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

%%%%%%%%%%%%%%%
% First form the modified Atilde, Btilde and Ctilde matrices to account for
% the direct feedthrough term arising from the D matrix. Tried to
% generalize for arbitrary input case. Time will tell if it worked lol.

Atilde = [A, B; zeros(size(B,2),size(A,2)), eye(size(B,2))];
Btilde = [zeros(size(B,1),1); ones(size(B,2),1)];
Ctilde = [C, D];

%%%%%%%%%%%%%%%
% X = M*x0 + S*DeltaU

M = [];
for i = 1:(Np)
    M = [M;Atilde^i];
end

col_S = [];
for i = 1:(Np)
    col_S = [col_S;(Atilde^(i - 1)*Btilde)];
end

S = zeros(size(Atilde,1)*(Np + 1),(Nc*size(Btilde,2)));

for i = 1:(Nc)
    if i == 1
        S = col_S;
    else
        col_S = [zeros(size(Btilde,1),1);col_S]; 
        col_S((end - size(Btilde,1)) + 1:end,:) = [];
        S = [S,col_S];
    end
end

%%%%%%%%%%%%%%%
% Y = Phi*Atilde*x0 + Gamma*DeltaU

Phi = [];
for i = 1:(Np)
    Phi = [Phi;Ctilde*Atilde^(i-1)];
end

% we actually need Phi*Atilde in the new formulation so spit that out of
% the function instead of just Phi

Phi_mult_Atilde = Phi*Atilde;

col_Gamma = [];
for i = 1:(Np)
    col_Gamma = [col_Gamma;Ctilde*(Atilde^(i - 1))*Btilde];
end

Gamma = zeros(size(Ctilde,1)*(Np + 1),(Nc*size(Btilde,2)));
for i = 1:(Nc)
    if i == 1
        Gamma = col_Gamma;
    else
        col_Gamma = [zeros(size(Ctilde,1),1);col_Gamma]; 
        col_Gamma((end - size(Ctilde,1)) + 1:end,:) = [];
        Gamma = [Gamma,col_Gamma];
    end
end

% Pluck out only SOC so that we can penalize the difference between the
% measured SOC and the targe SOC. Q bar is a symmetric & positive-semi
% definite matrix.
Q = diag([1, 0, 0, 0]);
Qbar = blkdiag(Q);

while size(Qbar) ~= Np*size(Ctilde,1)
    Qbar = blkdiag(Q,Qbar);
end


%%%%%%%%%%%%%%%
% Cost Function
SP = SP*ones(size(Gamma,1),1);

H = 2.*(Gamma'*Qbar*Gamma + r);
q = 2.*Gamma'*Qbar*(Phi_mult_Atilde*X0 - SP);

% we might need to move X0 outside the function since that's what we did
% for HW3 P2, but here we have two terms so we might have to output both if
% we go that route

%%%%%%%%%%%%%%%
% Handle constraints

% should the DeltaU constrain be Np x Nc or just Nc x Nc? If the control
% horizon is 3 and prediction horizon is 10, DeltaU will be constant for
% steps 4, 5, 6, 7, 8, 9, 10 so I think we only have to constrain the first
% three.. not sure though.
G = [S; -S; eye(Nc); -eye(Nc); Gamma; -Gamma];

Xmax = [];
Xmin = [];
DeltaUmax = [];
DeltaUmin = [];
Ymax = [];
Ymin = [];

while size(Xmin,1) ~= size(S,1)
    Xmax = [Xmax;xlimit.max];
    Xmin = [Xmin;xlimit.min];  
end

for i = 1:Nc
    DeltaUmax = [DeltaUmax;ulimit.max];
    DeltaUmin = [DeltaUmin;ulimit.min];
end 

while size(Ymin,1) ~= size(Gamma,1)
    Ymax = [Ymax;ylimit.max];
    Ymin = [Ymin;ylimit.min];
end 

W = [Xmax; -Xmin; DeltaUmax; -DeltaUmin; Ymax; -Ymin];
T = [-M; M; zeros(Nc,size(Atilde,1)); zeros(Nc,size(Atilde,1)); 
    -Phi_mult_Atilde; Phi_mult_Atilde];

IMPC = [1, zeros(1,(Nc-1))];

end
