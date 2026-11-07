function [F, Q] = singer_FQ(dt, tau_m, sigma_m)
% SINGER_FQ  Discrete Singer model for ONE axis, state [p; v; a].
%   [F, Q] = singer_FQ(dt, tau_m, sigma_m)
%   F : state transition over dt (your hand derivation, W1)
%   Q : covariance of the effect of all kicks inside dt (W3)
%
% Continuous model:  pdot = v,  vdot = a,  adot = -alpha*a + w,
%   w white with intensity q = 2*alpha*sigma_m^2   (from W2, Var(n) = sigma_m^2 (1-rho^2))
%
% Q = int_0^dt F(s) B q B' F(s)' ds  is evaluated exactly with Van Loan's
% trick (C. F. Van Loan, IEEE TAC 23(3), 1978; Brown & Hwang, Ch. 3):
%   M = [-A, BqB'; 0, A'] * dt,  G = expm(M),  F = G22',  Q = F * G12

alpha = 1/tau_m;
q     = 2*alpha*sigma_m^2;
A = [0 1 0; 0 0 1; 0 0 -alpha];
B = [0; 0; 1];

M = [-A, B*q*B'; zeros(3), A'] * dt;
G = expm(M);
F = G(4:6, 4:6)';
Q = F * G(1:3, 4:6);
Q = (Q + Q')/2;   % TRICK 2: remove round-off asymmetry so P stays a valid covariance
end
