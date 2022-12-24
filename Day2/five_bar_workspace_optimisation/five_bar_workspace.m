%% Example-2: Tool holder
% Formulate the workspace maximisation of a hand-held tool-holder as a
% design optimisation problem

% Write a function workspace_area.m to return the workpace area given
% the design variables 

% Design variables, dv(1) = l1, dv(2) = l3, dv(3) = d
% Design parameters, p(1) = a, p(2) = b, p(3) = L
p = [0.1, 0.4, 1];
x0 = [1,1,1];

% Draw the scenario corresponding to the chosen inintial guess and design
% parameters 

% Objective function definition
obj = @(dv)(-2*workspace_area(dv,p));

% Run the problem unbounded
opt = optimoptions('fminunc','Display','iter');
[dv,fval,exitflag,output] = fminunc(obj,x0,opt);
% Q1. What do you observe? What is the solution?

% Run the problems adding bounds on the design variables
lb = [0,0,max(p(1),p(2))];
ub = [p(3),p(3),+Inf];
opt = optimoptions('fmincon','Display','iter');
[dv,fval,exitflag,output] = fmincon(obj,x0,[],[],[1,1,0],p(3),lb,ub,[],opt);
% Q2. What do you observe? How does the solution change?

% Run the problem by adding constraints
% cons = ;
% opt = ;
% already implemented in the last question

% Q3: Interpreting the results as compared to the two-bar robot design
% Q3.1. Plot the configuration corresponding to the optimal values
figure;
d = dv(3);
l1 = dv(1);
l3 = dv(2);
a = p(1);
b = p(2);
theta = pi - acos(d/(2*p(3)));
theta1 = acos(d/(2*p(3)));
foval = @(x,y)(y^2/(a/2)^2+x^2/(b/2)^2-1);
fimplicit(foval);
% plot d
line([-d/2,d/2],[0,0],'color','r');
% plot l1 on the right side
line([d/2,d/2+l1*cos(theta)],[0,l1*sin(theta)],'color','b');

% plot l3 on the right side
line([d/2+l1*cos(theta),d/2+l1*cos(theta)+l3*cos(theta)],[l1*sin(theta),l1*sin(theta)+l3*sin(theta)],'color','g');

% plot l1 on the left side
line([-d/2,-d/2+l1*cos(theta1)],[0,l1*sin(theta1)],'color','b');

% plot l3 on the left side
line([-d/2+l1*cos(theta1),-d/2+l1*cos(theta1)+l3*cos(theta1)],[l1*sin(theta1),l1*sin(theta1)+l3*sin(theta1)],'color','g');
axis equal

% Q3.2. Explain the resuling optima (why the link lengths and the offsets are so)
% 1. the offset d equals the human waist width, because the closer are the
% two ends of the offset, the larger the overlapping area is, so is the
% workspace bigger.

% 2. the link lengths are equal to L/2, so that there are no vacant space
% inside of the workspace.

% Q4. Bonus: Considering collision between the robot links and the human
% ellipse as a non-linear constraint by checking intersection between links
% and the ellipse
  
% foval_explicit = @(x)(sqrt(1-4*a^2*x^2)/(4*b^2));
% diff(foval_explicit)
