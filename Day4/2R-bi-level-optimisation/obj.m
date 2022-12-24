function [J] = obj(w,init)
% The only input is the entire vector of the appended control and state
% Isolate only the control variables and compute the cost
% Get required variables from init
t0 = init.t0 ;
tf = init.tf ;
l1 = init.l1 ;
l2 = init.l2 ;
m1 = init.m1 ;
m2 = init.m2 ;
me = init.me ;
N = init.N;
nc = init.nc;
% Retrieve the list of control variables
u = w(1:N*nc);
% Compute the objective function value
J = 0;
for i = 1:size(u)
    J = J + u(i)^2;
end