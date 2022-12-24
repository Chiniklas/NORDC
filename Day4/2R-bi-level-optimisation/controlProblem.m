function [fval] = controlProblem(dv,init)
%CONTROLPROBLEM Solves the trajectory optimisation problem here
%   This is the lower level control problem for the upper level design
%   problem

% Since choosing mass and link lengths independantly does not make any
% sense anymore because --> Trivial solution m1, m2, m3->0
% Design parameters from the previous exercise turn into design variables
% for this exercise
init.l1 = dv(1);
init.l2 = dv(2);
init.r1 = dv(3); 
init.r2 = dv(4); 
init.m1 = init.rho * init.l1 * pi * init.r1^2 * 1e-3;
init.m2 = init.rho * init.l2 * pi * init.r2^2 * 1e-3;

N = init.N;
n = init.n;
nc = init.nc;
x0 = init.x0;
xf = init.xf;
%% Solve the lower level control sub-problem 
% Initial guess for all the discretised states
z_guess = zeros(N,n);
z_guess(1,:) = init.x0;
z_guess(N,:) = init.xf;
z_guess = z_guess';
z_guess = reshape(z_guess,n*N,1);
% Bounds on states
lb_z_x1 = -pi * ones(1,N);
lb_z_x2 = -10 * ones(1,N);
lb_z_x3 = -pi * ones(1,N);
lb_z_x4 = -10 * ones(1,N);
lb_z = [lb_z_x1;lb_z_x2;lb_z_x3;lb_z_x4];
lb_z = reshape(lb_z,[],1);
ub_z_x1 = pi * ones(N,1);
ub_z_x2 = 10 * ones(N,1);
ub_z_x3 = pi * ones(N,1);
ub_z_x4 = 10 * ones(N,1);
ub_z = [ub_z_x1;ub_z_x2;ub_z_x3;ub_z_x4];
ub_z = reshape(ub_z,[],1);
% Initial guess for all the discretised control
u_guess = zeros(N*nc,1);
w0 = [u_guess;z_guess];
% Bounds on control
lb_u = -10 * ones(N*nc,1);
ub_u = 10 * ones(N*nc,1);

lb = [lb_u;lb_z];
ub = [ub_u;ub_z];
%% Optimization setup

optNLP = optimset('TolX', 1e-5, 'TolFun', 1e-5, ...
    'MaxFunEval', 1e5, 'MaxIter', 1e3,'Display','off');

% Write a function to enforce constraint at the `knot points` in a
% function called `nlconst`
% Here is the same control problem you solved using fmincon
[opt, fval1] = fmincon(@(w)obj(w,init), w0, [], [], [], [], lb, ub, @(w)nonlcon(w,init), optNLP);

% We just have to return fval
% You can simply pass fval1 back or construct a meta objective function representing the total control
% energy, workspace and the cost of the links or more complicated ones
% using the structural strength etc.

fval = fval1;
end

