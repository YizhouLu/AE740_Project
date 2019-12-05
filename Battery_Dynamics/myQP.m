function [U, lam] = myQP(H, q, G, Wtilde, lam0)

% This function implements the dual projected gradient algorithm for
% solving a QP problem. Minimize 1/2*U'*H*U + q'*U subject to G*U <= Wtilde

invH = inv(H);
G_invH = G*invH; % see Note 1
Hd = G_invH*G';
qd = G_invH*q + Wtilde; 
Nit = 30;
lam = lam0;
L = norm(Hd);
k =1;
df = Hd*lam + qd;

while k <= Nit % see Note 2
    % maximum number of iterations
    lam = max(lam - 1/L*df, 0); 
    df = Hd*lam + qd;
    k =k+1;
end

U = -invH*(G'*lam + q);

return;

% In LQ-MPC setting, only q and Wtilde depend on x_0, makes no 
% sense to constantly re-compute these

end
