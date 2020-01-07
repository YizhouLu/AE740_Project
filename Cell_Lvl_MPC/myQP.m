function [dU, lam] = myQP(H, q, G, W_tilde, lam0, max_iter)
% Minimize 1/2*U'*H*U + q'*U
% subject to G * U <= W_tilt
invH = H^-1;
Hd = G * invH * G';
qd = G * invH * q + W_tilde;
lam = lam0;
L = norm(Hd);
num_iter = 1;
df = Hd * lam + qd;
while num_iter <= max_iter
    lam = max(lam - 1/L * df, 0);
    df = Hd * lam + qd;
    
    num_iter = num_iter + 1;
end
dU = -invH * (G'*lam + q);
end