%% Trajectory optimisation: Direct transcription
% Using trapezoidal collocation
clc
clear all
close all

% Start with the initial parameters of tf=5, l1=0.35, l2=0.3, m1=1.5
% m2 = 0.5, me=1;
init.t0 = 0;
init.tf = 5;
init.l1 = 0.35;
init.l2 = 0.3;
init.m1 = 1.5;
init.m2 = 0.5;
init.me = 1;
init.g = 9.8;
% Declare the parameters as done yesterday using the init struct
% Design parameters for control optimisation
% t0,tf,N,h,x0,xf,n(no. of states),m(no. of control inputs)

init.N = 50;
N = init.N;
init.h = (init.tf-init.t0)/init.N;
init.x0 = [0;0;0;0];
init.xf = [pi;0;0;0];
init.n = 4;
n = init.n;
init.m = 2;
m = init.m;
% Design parameters of the robot, gravity (g), l1, l2, m1, m2,
% me (payload)

% Set an initial guess for all the state values
%z_guess = zeros(N,n);
z_guess = rand(N,n);
z_guess(1,:) = init.x0;
z_guess(N,:) = init.xf;
z_guess = z_guess';
z_guess = reshape(z_guess,n*N,1);

% Set bounds on all the states
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

% Set initial guess for all control inputs 
u_guess = zeros(N*m,1);

w0 = [u_guess;z_guess];

% Set bounds on control
lb_u = -10 * ones(N*m,1);
ub_u = 10 * ones(N*m,1);

lb = [lb_u;lb_z];
ub = [ub_u;ub_z];

%% Optimization setup

% Setup options for the optimisation problem, read about `TolX`, `TolFun`,
% `MaxFunEval`, `MaxIter` and use them in the options
% Q1. How does modifying these variables effect the solution obtained?
optNLP = optimset('Display', 'iter','MaxFunEval',1e5,'TolX',1e-5,'TolFun',1e-5,'Algorithm','interior-point');
%optNLP = optimset('Display', 'iter','MaxIter',106,'Algorithm','active-set');
%optNLP = optimset('Display', 'iter','MaxFunEval',1e5,'TolX',1e-5,'TolFun',1e-5,'Algorithm','sqp');
% Write a function to enforce constraint at the `knot points` in a
% function called `nlconst`
% fun = @(w,init)obj(w,init);
tic
[opt, fval] = fmincon(@(w)obj(w,init),w0,[],[],[],[],lb,ub,@(w)nonlcon(w,init),optNLP);
t = toc;
%% Questions

% Q1. Solve the problem using fmincon and provide reasons for your observations
% Q2. Solve the problem using different algorithms available, `sqp`,
% `active-set`, `interior point` and note the following:
%     1. Number of iterations
%     2. Time taken
%     3. Number of function evaluations
%     4. Objective function value at optima
% and also the difference in the obtained solution by plotting the
% corresponding states and torques on the same plot

% Q3. How does the final solution change when:
%     1. tf is changed
%     2. N is changed
%     3. x0 is changed
%     4. xf is changed
%     5. g is changed
%     6. l1, l2, m1, m2 are changed
% and also the difference in the obtained solution by plotting the
% corresponding states and torques on the same plot

% Q4. Give a better initial guess for the states and see how the simulation
% time and iterations and the solution quality changes

% Q5. Modify the bounds on states and control and see how the behavior of
% the solution changes? can you find a solution for a small value of
% control bounds?

% Q6(Bonus). Modify the discretisation scheme from Euler to
% Hermite-Simphson or another and see how system behavior changes

% Q7(Bonus). Give the obtained control input to the system and observe the 
% system performance? Does the robot actually reach the final position as expected? 

%% Plotting the optimal control and states

% Extract optimal values
tempu = opt(1:init.m*init.N);
u = reshape(tempu,init.m,init.N)';
u(end+1,:) = u(end,:);

tempz = opt(init.m*init.N+1:end);
z = reshape(tempz,init.n,init.N)';
z(end+1,:) = z(end,:);

% Plotting the results
t = init.t0:init.h:init.tf;      % Discretize time based on step size
% Plotting the control actions
figure();
stairs(t,u(:,1));
xlabel('Time steps','FontSize', 12);
ylabel('Control input u','FontSize', 12);
hold on
stairs(t,u(:,2));

% plotting the states
figure();
plot(t,z(:,1),'b');
hold on;
plot(t,z(:,2),'r');
xlabel('Time steps','FontSize', 12);
ylabel('States','FontSize', 12);
plot(t,z(:,3),'--b');
plot(t,z(:,4),'--r');