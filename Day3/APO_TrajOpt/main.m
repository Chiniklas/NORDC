%% Trajectory optimisation: Direct transcription
% Using trapezoidal collocation
clc
clear all
close all

% Initialise design parameters in init
% Start with the initial parameters of tf=4
% Set t0, tf, x0, xf, N, t,n,nc,h
init.t0 = 0;
init.tf = 4;
init.x0 = [0;0;0;0;0;0];
init.xf = [0;0;0;pi;pi;0];
init.N = 50;
N = init.N;
init.t = 0;
init.n = 6;
init.nc = 3;
init.h = (init.tf-init.t0) / init.N;
init.l1 = 350;
init.l2 = 320;

t0 = init.t0;
tf = init.tf;
x0 = init.x0;
xf = init.xf;

n = init.n;
nc = init.nc; 
h = init.h; 
init.t = t0:h:tf;
%t = init.t;
% Set an initial guess for all the state values
z_guess = zeros(n,N);
z_guess(:,1) = x0;
z_guess(:,N) = xf;
z_guess = reshape(z_guess,[],1);
% Set bounds on all the states
lb_z_x1 = -pi * ones(1,N);
lb_z_x2 = -10 * ones(1,N);
lb_z_x3 = -pi * ones(1,N);
lb_z_x4 = -10 * ones(1,N);
lb_z_x5 = -pi * ones(1,N);
lb_z_x6 = -10 * ones(1,N);
lb_z = [lb_z_x1;lb_z_x2;lb_z_x3;lb_z_x4;lb_z_x5;lb_z_x6];
lb_z = reshape(lb_z,[],1);
ub_z_x1 = pi * ones(N,1);
ub_z_x2 = 10 * ones(N,1);
ub_z_x3 = pi * ones(N,1);
ub_z_x4 = 10 * ones(N,1);
ub_z_x5 = pi * ones(N,1);
ub_z_x6 = 10 * ones(N,1);
ub_z = [ub_z_x1;ub_z_x2;ub_z_x3;ub_z_x4;ub_z_x5;ub_z_x6];
ub_z = reshape(ub_z,[],1);
% Set initial guess for all control inputs 
u_guess = zeros(N*nc,1);
w0 = [u_guess;z_guess];
% Set bounds on control
lb_u = -10 * ones(N*nc,1);
ub_u = 10 * ones(N*nc,1);

lb = [lb_u;lb_z];
ub = [ub_u;ub_z];

%% Optimization setup

% Setup options for the optimisation problem, read about `TolX`, `TolFun`,
% `MaxFunEval`, `MaxIter` and use them in the options
% Q1. How does modifying these variables effect the solution obtained?
optNLP = optimset('Display', 'iter','MaxFunEval',1e5,'TolX',1e-5,'TolFun',1e-5,'Algorithm','interior-point');

% Write a function to enforce constraint at the `knot points` in a
% function called `nlconst`

[opt, fval] = fmincon(@(w)obj(w,init),w0,[],[],[],[],lb,ub,@(w)const(w,init),optNLP);
%% Visualising the results
% Q1. Plot the resulting states and control input 
% Extract optimal values
tempu = opt(1:nc*N);
u = reshape(tempu,nc,N)';
u(end+1,:) = u(end,:);

tempz = opt(nc*N+1:end);
z = reshape(tempz,n,N)';
z(end+1,:) = z(end,:);

% Plotting the results
t = t0:h:tf;      % Discretize time based on step size
% Plotting the control actions
figure();
stairs(t,u(:,1),'b');
xlabel('Time steps','FontSize', 12);
ylabel('Control input u','FontSize', 12);
hold on
stairs(t,u(:,2),'r');
hold on
stairs(t,u(:,3),'g');

% plotting the states
figure();
plot(t,z(:,1),'b');
hold on;
plot(t,z(:,2),'r');
plot(t,z(:,3),'g');
xlabel('Time steps','FontSize', 12);
ylabel('States','FontSize', 12);
plot(t,z(:,4),'--b');
plot(t,z(:,5),'--r');
plot(t,z(:,6),'--g');
% Q2. Write a code similar to `robolinplot` to visualise the resulting
% trajectory of the musculo-skeletal robot
% Q3. Repeat Q1-Q2, Q3.1-3.4, Q4, Q5 from the previous problem