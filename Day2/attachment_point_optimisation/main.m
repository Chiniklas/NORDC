%%% Consistent Unit System    %%%
%%% ton, mm, s , N , MPa, Nmm %%%

close all
clear all
clc

% Use structs to initialise the problem, i.e., init.name = bla form
% Initialization, `linklength`, `ballradius`, `armradius`, `ballcenter`, `range`,
% `nTendons`, `nSamples`, `domainRadius`

init.linklength = 350;
init.ballradius = 20;
init.armradius = 10;
init.ballcenter = [0;0;0];
init.nTendons = 4; 
init.domainRadius = 60;
init.nSamples = 5;
init.range = -90:5:90;
 
% Number of design variables (y-z coordinates of the attachment points)
nvars = 8; %4 attachment points each has a y and a z coordinate

% Setup lower and upper bounds for the problem for all the design variables
% Hint: Use polar co-ordinates instead of Cartesian coordinates as design
% variables, i.e., angle and radius of the each points, 
% dv = [angle1, length1, angle2, length2, angle3, length3, angle4, length4];
lb_l = init.ballradius;
ub_l = init.domainRadius;
lb = [0, lb_l, 0, lb_l, 0, lb_l, 0, lb_l];
ub = [2*pi, ub_l, 2*pi, ub_l, 2*pi, ub_l, 2*pi, ub_l];

% Declare the objective function `checkscore` with parameters init
objfn = @(x)(checkscore(x,init));

% Solve the upper-level optimisation problem using particle swarm
tic
options = optimoptions("particleswarm",'SwarmSize',50,'Display','iter','FunctionTolerance',1e-5);
[x,fval,exitflag,output] = particleswarm(objfn,nvars,lb,ub,options);
t = toc;
% 
x;
fval;


%% Questions:
% Q1. Draw the flow-chart for the bi-level optimisation problem
% Q2. Draw ADG for the optimisation problem
% Q3. Plot objective function value vs iterations of the optimisation
% Q4. Qualitatively how do you choose the termination iterations
% Q5. What are the optimal values for the design variables? Add an image of
% the cables and the link. 
% Q6. Modify the swarm size with atleast two different values and repeat the experiment and document how it
% affects the optimal value and the computation time. 
% Q7. What are the function and convergance tolerance for the chosen
% algorithm in the linear programming and how does it affect the solution?
% (Choose atleast two different values and document the iterations, time taken and the solution)


%% -------------------------------------- 
% Do not edit blow this
%  -------------------------------------- 

% Plotting the obtained solution
%Init
linklength = init.linklength;
armradius = init.armradius;
range = init.range;
nTendons = init.nTendons;

score = 0;
count = 0;
forcemaxavg = 0;

A = zeros(3,nTendons);
armend0 = [linklength;0;0];
B0 = [linklength,linklength,linklength,linklength;
      0,        - armradius,  0      , armradius
      armradius,    0      , -armradius, 0  ];
  

[y,z]  = pol2cart(x(1:2:nvars),x(2:2:nvars)); 

range = init.range;
nSamples = length(range)^2;  %Increasing nSamples for final workspace calculation
workspace = zeros(3,nSamples);
finalpoints = zeros(3,nSamples);
i=1;
for thetay=range
    for thetaz =range

    %Gives you the transformed coordinates
    B = transform(B0,0,thetay,thetaz);
    armend = transform(armend0,0,thetay,thetaz);

    for j=1:nTendons
        A(:,j) = [0; y(j); z(j)];
        l = (A(:,j) - B(:,j))/norm(A(:,j) - B(:,j));
        jt(:, j) = cross(B(:,j), l);
    end 
    
    %Prevent numerical errors, everything smaller then 1e-3 is zero
    jt(1e-3>jt & jt >-1e-3)=0;

    % For now just 4 single load cases, no Torsion
    nLoadcases = 4;
    G = [0;20;0;...
         0;0;20;...
         0;-20;0;...
         0;0;-20];


    %4 Tendons, and 4 Loadcases, therefore f = [24,1]
    f = ones(nTendons*nLoadcases,1);

    % Keep the tension positive
    lb = 1e-1*ones(nTendons*nLoadcases,1);
    ub = 3000*ones(nTendons*nLoadcases,1);

    %JT consists now on independent blocks of jt
    %Each sub problem has it own design variables for the forces
    JT = zeros(size(jt,1)*nLoadcases,size(jt,2)*nLoadcases);
    for j=1:nLoadcases
         idx1 = (j-1)*size(jt,1)+1;
         idx2 = j*size(jt,1);
         idx3 = (j-1)*size(jt,2)+1;
         idx4 =  j*size(jt,2);
         JT(idx1:idx2,idx3:idx4) = jt;
    end 
    options = optimoptions('linprog','Algorithm','interior-point');
    options.Display = 'off';
    [force, ~, exitflag] = linprog(f, [], [], JT, G, lb, ub, options);

    if exitflag == 1 
        if cable_balljoint_collision(A, B, init) == 1 && cable_cable_collision(A,B,init)==1
        %if cable_balljoint_collision(A, B, init) == 1
            workspace(:,i) = [0;thetay;thetaz];
            finalpoints(:,i) = transform(armend0,0,thetay,thetaz);
            count = count + 1/nSamples;
        end
    end
    i=i+1;

    end
end 

count
figure(1)
scatter3(finalpoints(1,:),finalpoints(2,:),finalpoints(3,:));
xlabel('X');
ylabel('Y');
zlabel('Z');

figure(2)
lb = -60*ones(1,nvars);
ub = 60*ones(1,nvars);
plotconfiguration(A,B0,init)

function plotconfiguration(A,B,init)

    [Z,Y,X]=cylinder(init.domainRadius);
    surf(0*X,Y,Z);
    hold on
    for j=1:init.nTendons
        plot3( [A(1,j),B(1,j)],[A(2,j),B(2,j)],[ A(3,j),B(3,j)],'r')
        hold on
    end
  
    [Z,Y,X]=cylinder(init.armradius);
    surf(X*init.linklength,Y,Z);
    hold on
    
    [X,Y,Z]=sphere();
    r = init.ballradius;
    X = X*r;
    Y = Y*r;
    Z = Z*r;
    surf(X,Y,Z)
end

