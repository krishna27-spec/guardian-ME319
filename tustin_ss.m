function K = tustin_ss(A, B, C, D, h)
% TUSTIN_SS  Bilinear (Tustin) discretisation of a state-space system,
% s = (2/h)(z-1)/(z+1). Same transfer function as c2d(...,'tustin');
% the state is scaled differently, which does not matter for the controller.
I  = eye(size(A));
Dl = inv(I - A*h/2);
K.Ad = Dl*(I + A*h/2);
K.Bd = Dl*B*h;
K.Cd = C*Dl;
K.Dd = D + C*Dl*B*h/2;
end
