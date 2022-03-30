function [x_next] = RK4(X,U,quad)
%
% Inputs : 
%    X, U current state and input
%    h    sample period
%    f    continuous time dynamics f(x,u)
% Returns
%    State h seconds in the future accprding to RK
%

% Runge-Kutta 4 integration
% write your function here

   k1 = quad.f(X,        U);
   k2 = quad.f(X+quad.Ts/2*k1, U);
   k3 = quad.f(X+quad.Ts/2*k2, U);
   k4 = quad.f(X+quad.Ts*k3,   U);
   x_next = X + quad.Ts/6*(k1+2*k2+2*k3+k4);
   
   

end