function Bfinal = transform(B,t11,t12,t13)

% This function converts a transforms a given matrix by the given set of
% rotation angles
% Convert degrees to radians
t11 = deg2rad(t11);
t12 = deg2rad(t12);
t13 = deg2rad(t13);

% Set of vectors to be transformed
B = [B;ones(1,size(B,2))];

%Coordinate Transformations
T0 = [0 0 1 0;...
      0 1 0 0;...
     -1 0 0 0;...
      0 0 0 1];

% Homogenous transformation wrt X-axis
T1 = [cos(t11) -sin(t11) 0 0;...
      sin(t11) cos(t11) 0 0;...
      0 0 1 0;...
      0 0 0 1];
% Homogenous transformation wrt Y-axis
T2 = [cos(t12-pi/2) -sin(t12-pi/2) 0 0;...
      0 0 1 0;...
      -sin(t12-pi/2) -cos(t12-pi/2) 0 0;...
      0 0 0 1]; 

% Homogenous transformation wrt Z-axis
T3 = [cos(t13) -sin(t13) 0 0;...
      0 0 -1 0;...
      sin(t13) cos(t13) 0 0;...
      0 0 0 1]; 

%% Final transformation
T13 = T0*T1*T2*T3;

Bfinal = T13*B;
Bfinal = Bfinal(1:3,:);

end

