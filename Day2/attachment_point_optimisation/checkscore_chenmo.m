function score = checkscore(x,init)

%Init all params here to use as individual variables
% Example, bla = init.bla;
disp(x);
% Start initial values for the objective function value
score = 0;
count = 0;
forcemaxavg = 0;

% Coordinates of the cables attached on the other end concatenated in a 3x4
% matrix in the home position
B0 = [init.linklength,init.linklength,init.linklength,init.linklength;
      0,        - init.armradius,  0      , init.armradius
      init.armradius,    0      , -init.armradius, 0  ];
% Co-ordinates of the armend
armend0 = [init.linklength;0;0];

% Sample random angles in the given range
thetax = -5 + (5+5)*rand(1,init.nSamples);
thetay = -5 + (5+5)*rand(1,init.nSamples);
thetaz = -5 + (5+5)*rand(1,init.nSamples);

% Read and use the built-in function `pol2cart` to convert the DVs in polar
% coordinates to Cartesian co-ordinates
nvars = 8;
[y,z]  = pol2cart(x(1:2:nvars),x(2:2:nvars)); 

% Iterate over all the samples 
for i=1:init.nSamples
    % Start with one pose of all the samples

    % Complete the function to transform coordinates between frames for a sampled
    % design and point in the workspace in the file called `transform`
    % Get the transformed coordinates for all the end points of the cables
    B = transform(B0,0,thetay(i),thetaz(i));
    % Get the transformed coordinates for all the end points of the arm
    armend = transform(armend0,0,thetay(i),thetaz(i));
    
    % Create a matrix of size 3,nTendons with all elements as zeros
    A = zeros(3,init.nTendons);% force on cable

    % Find the direction vectors of cable lengths
    for j=1:init.nTendons
        A(:,j) = [0;y(j);z(j)];
        l = (A(:,j))/norm(A(:,j));%force direction
        % Find the unit moment between due to the cables, i.e., cross
        % between B and l and append it in jt
        jt(:,j) = cross(B(:,j),l);
    end 
    %Prevent numerical errors, everything smaller then 1e-3 is zero
    jt(1e-3>jt & jt >-1e-3)=0;
    if sum(sum(isnan(jt)))>0
        continue;
    end
    %disp(jt);

    % For now consider 4 single load cases, with no torsion applied at the
    % end-effector 
    % Convert the load cases to moments about the base point
    nLoadcases = 4;
    % Define load cases
    G = [0;20;0;...
         0;0;20;...
         0;-20;0;...
         0;0;-20];

    % Since the problem of finding the forces is independant for each of
    % the load cases, we can construct one bigger problem instead for all
    % the load cases combined instead

    % 4 Tendons, and 4 Loadcases, therefore f is a vector of ones of size [16,1]
    % Initial guess for the forces
    f = ones(init.nTendons*nLoadcases,1);
    
    % Ensure that the cables retain minimum positive tension
    lb = 1e-1*ones(init.nTendons*nLoadcases,1);
    ub = 3000*ones(init.nTendons*nLoadcases,1);

    %JT consists now on independent blocks of jt
    %So declare a matrix JT such that jts are diagonal elements of JT
    JT = zeros(size(jt,1)*nLoadcases,size(jt,2)*nLoadcases);
    for j=1:nLoadcases
         idx1 = (j-1)*size(jt,1)+1;
         idx2 = j*size(jt,1);
         idx3 = (j-1)*size(jt,2)+1;
         idx4 =  j*size(jt,2);
         JT(idx1:idx2,idx3:idx4) = jt;
    end 
    % Set options to solve the inner linear programming problem
    options = optimoptions('linprog','Algorithm','interior-point');
    options.Display = 'off';

    % Solve the feasibility problem using MATLABs `linprog` function
    %disp(JT);
    [force,~, exitflag] = linprog(f,[],[],JT,G,lb,ub,options);

    % Get feasibility with respect to the constraints (use similar idea for
    % the bonus question of the previous problem)
    if exitflag == 1 
        % Write two functions `cable_balljoint_collision` and
        % `cable_cable_collision` to check if the current design has
        % collisions between cable-cable or cable-ball-joint
        % Hint: Start with one check at a time
%         if cable_balljoint_collision() == 1 && cable_cable_collision()==1
%             ;
%         end
        if all(force>0)
            count=count+1;
            forcemaxavg=forcemaxavg*(count-1)/count+sum(force)/count;
        end
    
    end
end

% Formulate a scoring factor to represent how good a design is
score = count/init.nSamples

end 

