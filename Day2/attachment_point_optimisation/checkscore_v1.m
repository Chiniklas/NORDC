function score = checkscore(x,init)

% Init all params here to use as individual variables
% Example, bla = init.bla;

% Start initial values for the objective function value
score = 0; 
count = 0;
forcemaxavg = 0;
linklength = init.linklength;
armradius = init.armradius;
range = init.range;
nTendons = init.nTendons;
nSamples = length(range)^2;
nvars = 8;
% Coordinates of the cables attached on the other end concatenated in a 3x4
% matrix in the home position
B0 = [linklength,linklength,linklength,linklength;
      0,        - armradius,  0      , armradius
      armradius,    0      , -armradius, 0  ];
% Co-ordinates of the armend
armend0 = [linklength;0;0];

% Sample random angles in the given range
thetax = -5 + (5+5)*rand(1,nSamples);
thetay = -5 + (5+5)*rand(1,nSamples);
thetaz = -5 + (5+5)*rand(1,nSamples);

% Read a+s in polar
% coordinates to Cartesian co-ordinates
[y,z]  = pol2cart(x(1:2:nvars),x(2:2:nvars)); 

% Iterate over all the samples 
for i=1:nSamples
    % Start with one pose of all the samples

    % Complete the function to transform coordinates between frames for a sampled
    % design and point in the workspace in the file called `transform`
    % Get the transformed coordinates for all the end points of the cables
    B = transform(B0,0,thetay(i),thetaz(i));
    % Get the transformed coordinates for all the end points of the arm
    armend = transform(armend0,0,thetay(i),thetaz(i));
    
    % Create a matrix of size 3,nTendons wi/lth all elements as zeros
    A = zeros(3,nTendons);

    % Find the direction vectors of cable lengths
    for j=1:nTendons
        A(:,j) = [0;y(j);z(j)];
        l = (A(:,j) - B(:,j))/norm(A(:,j) - B(:,j));
        % Find the unit moment between due to the cables, i.e., cross
        % between B and l and append it in jt
        jt(:,j) = cross(B(:,j),l);
    end 
    %Prevent numerical errors, everything smaller then 1e-3 is zero
    jt(1e-3>jt & jt >-1e-3)=0;

    % For now consider 4 single load cases, with no torsion applied at the
    % end-effector 
    % Convert the load cases to moments about the base point
    nLoadcases = 4;
    % Define load cases in one Matrix
    G = [0;20;0;...
         0;0;20;...
         0;-20;0;...
         0;0;-20];

    % Since the problem of finding the forces is independant for each of
    % the load cases, we can construct one bigger problem instead for all
    % the load cases combined instead

    % 4 Tendons, and 4 Loadcases, therefore f is a vector of ones of size [16,1]
    % Initial guess for the forces
    f = ones(nTendons*nLoadcases,1);
    
    % Ensure that the cables retain minimum positive tension
    lb = 1e-1*ones(nTendons*nLoadcases,1);
    ub = 3000*ones(nTendons*nLoadcases,1);

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
    [force,~, exitflag] = linprog(f, [], [], JT, G, lb, ub, options);
    forcemaxavg = forcemaxavg + force;
    % Get feasibility with respect to the constraints (use similar idea for
    % the bonus question of the previous problem)
    if exitflag == 1 
        % Write two functions `cable_balljoint_collision` and
        % `cable_cable_collision` to check if the current design has
        % collisions between cable-cable or cable-ball-joint
        % Hint: Start with one check at a time
%         if cable_balljoint_collision(A, B, init) == 1 && cable_cable_collision()==1
%             workspace(:,i) = [0;thetay;thetaz];
%             finalpoints(:,i) = transform(armend0,0,thetay,thetaz);
%             count = count + 1/nSamples;
%         end
        if all(foice>0)
            count= count+1;
            forcemaxavg=forcemaxavg*(count-1)/count+sum(force)/count;
        end
    end
end

% Formulate a scoring factor to represent how good a design is
score = count/nSamples;

end 

