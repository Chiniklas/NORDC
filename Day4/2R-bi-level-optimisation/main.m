%% Trajectory optimisation: Direct transcription
% Using trapezoidal collocation
clc
clear all
close all

% Parameters for control optimization
% Will be hyper-parameters for the bi-level optimisation
% Initial time
% Final time
init.t0 = 0;
init.tf = 5;
% Declare the other variables within an `init` struct
% Total number of steps
init.N = 10;
% Step size
init.h = (init.tf-init.t0)/init.N;
init.x0 = [0;0;0;0];   % Initial state q,qdot
init.xf = [pi/10;0;0;0];  % Final state
% Number of state variables
init.n = 4;
% Number of control inputs
init.nc = 2;
% Gravity and the payload
init.g = 9.8;
init.me = 1;

% Since we want to optimise the links design, we need link lengths and mass
% for control optimisation. For mass we assume the link is cylindrical and
% is made out of plastic whose material density is given below

% Declare density of the materials
init.rho = 1050; % Plastic

%% Posing the higher level design problem
% dv = [l1, l2, r1, r2]
% m1 = l1 * pi * r1^2;
% m2 = l2 * pi * r2^2;
nVars = 4;
lb = [0.2,0.2,0.05,0.05];
ub = [0.5,0.5,0.2,0.2];

% Define objective function
% Add the already completed control optimisation as a lower level 
% optimisation problem objective function
objfun = @(dv)controlProblem(dv, init);

% Solve the problem using particleswarm optimisation
% tic
% options = optimoptions('particleswarm','Display','iter','SwarmSize',20,'MaxIterations',20);
% [dv1,fval,exitflag,output] = particleswarm(objfun,nVars,lb,ub,options);
% toc

 
%% With constraints
% You can add the constraints from the previous design problem here
% But particle swarm does not admit any constraints, so switch back to
% fmincon
% Add an initial guess 
dv0 = [0.35;0.3;0.1;0.1];
options1 = optimoptions('fmincon','Display','iter');
[dv2,fval,exitflag,output] = fmincon(objfun,dv0,[],[],[],[],lb,ub,[],options1);
%[dv2,fval,exitflag,output] = fmincon(objfun,dv0,[],[],[],[],[],[],[],options1);
%% Once you are done with the optimisation use the above values in the 
% control optimisation one more time and visualise the robot using `robolinplot`
robolinplot