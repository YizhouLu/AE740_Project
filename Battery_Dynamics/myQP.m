function [DeltaU, lambda] = myQP(H,q,G,Wtilde,lam0)

% This function implements the dual projected gradient algorithm for
% solving a QP problem. Minimize 1/2*U'*H*U + q'*U subject to G*U <= Wtilde

% set the initial guess of lagrange multipliers.
lambda = lam0;

% form the quadratic term with respect to lambda of the gradient of the
% cost function with respect to DeltaU -> this is the Hessian of the dual
% problem and is multiplied by the lagrange multipliers
H_dual = G*inv(H)*G';

% form the linear term with respect to lambda of the gradient of the cost
% function with respect to DeltaU
q_dual = G*inv(H)*q + Wtilde; 

% define the norm of the dual Hessian to be used for projection on the
% non-negative orthant
L = norm(H_dual);

% define the derivative of the dual cost function with respect to lambda
dual_grad = H_dual*lambda + q_dual;

% initialize the algorithm and set the max number of iterations
k = 1;
Nit = 1000;

while k <= Nit
    lambda = max(lambda - (1/L)*dual_grad, 0); 
    dual_grad = H_dual*lambda + q_dual;
    k = k+1;
end

DeltaU = -inv(H)*(G'*lambda + q);

return;

end
