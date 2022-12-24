%% -----------------------------
% Getting started with Matlab 
% -----------------------------

%% Writing functions
% Matlab needs a function with the same name as the function
% Write a functions to add two numbers by creating a new `function` in the
% current folder

%% Declare a function just using a function handle
% See documentation about function handles and declare the adder just using
% a function handle
% Use function handle to write a function to add two numbers
fun = @add;

%% Read the documentation of the optimisers discussed during the class
% SQP
% SLP
% Particle Swarm
% Active-set strategy
% Bonus: Interior point algorithms

% Hint: have a look at fmincon and fminunc

% What are the inputs and outputs required? 
% How is the objective function defined
% How are the constraints provided? 
% What are the options and what do they do? 

%% Tryout an example optimisation problem using the above methods
% Consider Himmelblau's function
f = @(x,y)(x.^2+y-11).^2+(x+y.^2-7).^2;

% Visualise the surface of the function for x, y range [-5,5]
x = -5:0.1:5;
y = -5:0.1:5;
[X,Y] = meshgrid(x,y);
Z = f(X,Y);
surface_plot = surf(X,Y,Z);

% Draw the contours of the function
figure;
cnt_plot = contour(X,Y,Z);

%% Notice the following for the optimisation problems to be solved
% The number of iterations
% The total functional evaluations required
% Reason for convergance
% Time taken 
% Behavior on changing the initial guess
% Convergance behavior
% Satisfaction of constraints
% Value of the objective function
% Effect of availability of gradients

%% Unconstrained optimisation
% Find the minima of the Himmelblau's function assuming no other
% constraints
f = @(x)(x(1).^2 + x(2) - 11).^2 + (x(1) + x(2).^2 - 7).^2; 
% What kind of optimisation problem are we dealing with?

% How do you choose an initial guess?
x0 = [1,1];

% How do you choose the algorithm?
% https://de.mathworks.com/help/optim/ug/choosing-the-algorithm.html

% Solving the problem usign quasi-newton algorithm (use options)
options = optimoptions('fminunc','Display','iter','Algorithm','quasi-newton');
[x,fval,exitflag,output] = fminunc(f,x0,options);

%Providing gradient of the function
options = optimoptions('fminunc','Algorithm','quasi-newton');
[x,fval,exitflag,output] = fminunc(@himmelblauwithgrad,x0,options);

% Read about `CheckGradients` option and see what it does
options = optimoptions('fminunc','Algorithm','quasi-newton','CheckGradients', true);
[x,fval,exitflag,output] = fminunc(@himmelblauwithgrad,x0,options);

% Now try trust region algorithm with and without gradients
options = optimoptions('fminunc','Algorithm','trust-region','SpecifyObjectiveGradient',true);
[x,fval,exitflag,output] = fminunc(@himmelblauwithgrad,x0,options);

% Some methods need the gradients otherwise they do not work. See how trust
% region works
% options = optimoptions('fminunc','Algorithm','trust-region','SpecifyObjectiveGradient',true);
% [x,fval,exitflag,output] = fminunc(f,x0,options);

%% What happens when we only have a budget for 2 iterations, with and
% without gradients using quasi-newton
options = optimoptions('fminunc','Algorithm','quasi-newton','MaxIterations',2);
[xout,fval,exitflag,output] = fminunc(f,x0,options);

% Using tic-toc to find the time taken for the code snippet
tic
options = optimoptions('fminunc','Algorithm','quasi-newton','MaxIterations',2);
[xout,fval,exitflag,output] = fminunc(f,x0,options);
toc

% See other available options, especially try to use `UseParallel`

%% Constrained optimisation
% Solving the same problem in the presense of constraints
% Consider the following linear constraint, A*x - b, x(1)+2*x(2)<=1
% Read the documentation of fmincon and solve this constrained optimisation
% problem
A = [1,2];
b = 1;
options = optimoptions(@fmincon,'Display','iter');
[x,fval,exitflag,output] = fmincon(f,x0,A,b,[],[],[],[],[],options);

% Consider the bounds on the variables x = [-3,0], y = [-0.5,0.5]
lb = [-3,-0.5];
ub = [0,0.5];
options = optimoptions(@fmincon,'Display','iter');
[x,fval,exitflag,output] = fmincon(f,x0,A,b,[],[],lb,ub,[],options);

% Plot all the optima on the contour plots

% Consider the following non-constraint
g = @(x)((x(1)+3).^2+x(2).^2)-4;
[x,fval,exitflag,output] = fmincon(f,x0,A,b,[],[],lb,ub,@nonlcon,options);

% % What is the error. Refer to the documentation and reformulate it in
% % function named ncon_himmelblau
% nonlcon1 = @ncon_himmelblau;
% [x,fval,exitflag,output] = fmincon(f,x0,A,b,[],[],lb,ub,nonlcon1,options);

% Plot the constraints and the solutions

%% Different algorithms for fmincon
% SQP - Notice the warning MATLAB gives for this command, with the given x0
options = optimoptions(@fmincon,'Algorithm','sqp');
[x,fval,exitflag,output] = fmincon(f,x0,A,b,[],[],lb,ub,[],options);

% active-set
options = optimoptions(@fmincon,'Algorithm','active-set');
[x,fval,exitflag,output] = fmincon(f,x0,A,b,[],[],lb,ub,[],options);

% What does the line search value mean as per what we discussed?
% Compare the iterations, functional evaluations and time taken when
% compared to an unconstrained problem

%% Using evolutionary algorithms
% https://de.mathworks.com/help/gads/constrained-minimization-using-ga.html

% Use ga to solve the problem with and without bounds
nvars = 2;
options = optimoptions(@ga,'Display','iter');
[x,fval] = ga(f,nvars);

% With constraints and bounds
[x,fval] = ga(f,nvars,A,b,[],[],lb,ub,[],options);
 

%% Using particle swarm
% https://de.mathworks.com/help/gads/particleswarm.html

% Use particle swarm optimiser to solve the problem
options = optimoptions(@particleswarm,'Display','iter');
[x,fval,exitflag,output] = particleswarm(f,nvars);

%% Interactive learning using the MATLAB live script editor, `Optimize` block
% https://de.mathworks.com/help/optim/ug/optimize.html


%% More examples given here:
% https://de.mathworks.com/help/optim/ug/fmincon.html and
% https://de.mathworks.com/help/optim/ug/fminunc.html to practise

% You can learn more about the behavior of these algorithms by trying them
% out on test functions: https://en.wikipedia.org/wiki/Test_functions_for_optimization