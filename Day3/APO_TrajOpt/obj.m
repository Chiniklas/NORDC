%% Objective Function
% minimise control energy
function [J] = obj(w, init)
    nc = init.nc;
    N = init.N;
    u = w(1:N*nc);
    J = sum(u.*u);
end