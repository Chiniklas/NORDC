dv = dv2;
init.l1 = dv(1);
init.l2 = dv(2);
init.r1 = dv(3); 
init.r2 = dv(4); 
init.m1 = init.rho * init.l1 * pi * init.r1^2 * 1e-3;
init.m2 = init.rho * init.l2 * pi * init.r2^2 * 1e-3;


init.m1 = 1.5;
init.m2 = 0.5;

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
    'MaxFunEval', 1e5, 'MaxIter', 1e3,'Display','iter');

% Write a function to enforce constraint at the `knot points` in a
% function called `nlconst`
% Here is the same control problem you solved using fmincon
[opt, fval1] = fmincon(@(w)obj(w,init), w0, [], [], [], [], lb, ub, @(w)nonlcon(w,init), optNLP);
%% draw state and control graph
% Extract optimal values
tempu = opt(1:init.nc*init.N);
u = reshape(tempu,init.nc,init.N)';
u(end+1,:) = u(end,:);

tempz = opt(init.nc*init.N+1:end);
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
%% draw robot
th1 = z(:,1);
th2 = z(:,2);
%read l1 and l2 from the previous code
l1 = init.l1;
l2 = init.l2;
plot_bounds = l1+l2+0.1;

%The coordinates of the links of each of the manipulator
x1 = l1*cos(th1);
y1 = l1*sin(th1);
x2 = l1*cos(th1)+l2*cos(th1+th2);
y2 = l1*sin(th1)+l2*sin(th1+th2);
figHandle = figure();
for i=1:init.N
    A = [0 x1(i)]; 
    B = [0 y1(i)]; 
    %   subplot(1,3,1);
    plot(A,B,'*')
    axis([-plot_bounds plot_bounds -plot_bounds plot_bounds])
    hold on
    line(A,B)
    hold on
    A2 = [x1(i) x2(i)]; 
    B2 = [y1(i) y2(i)];
    %   subplot(1,3,1);
    plot(A2,B2,'*')
    axis([-plot_bounds plot_bounds -plot_bounds plot_bounds])
    hold on
    line(A2,B2,'Color','red')
    pause(0.1);
end