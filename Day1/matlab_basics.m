%% -----------------------------
% Getting started with Matlab 
% -----------------------------

%% Declaring variables
% Declare and initiate two variables named `a` and `b`
% as a unit vector along x-axis and y-axis
a = [1;0;0];
b = [0;1;0];
% Transpose of the constructed vector a
a_transpose = a';

%%  Concatenating variables
% Concatenate the variables `a` and `b` along the same axis (as one column)
c1 = [a;b];
% Concatenate the variables `a` and `b` as two columns
c2 = [a,b];

%%  Using the colon operator
% Similar to linspace in python. Declaring a list of numbers between two specified number and a step
% Get list of numbers between 1 and 10 with 0.5 spacing
c3 = 1:0.5:10;

% Using a list using linspace between numbers 1 and 10 with step size of 1
c4 = 1:10;

%% Using predefined matrices
% Identity matrix of size 4
I1 = eye(4);

%  One dimensional column vector of length 7 and all elements as 1
c5 = ones(7,1);

% One dimensional row vector of length 3 and all elements as 0
c6 = zeros(3,1);

%% Matrix manipulation
A = [1,2,3;4,5,6;7,8,9];
B = [-1,2,-3;4,-5,6;-7,8,-9];

% Multiplying two matrices
C1 = A * B;

% Element wise multiplication
C2 = A .* B;

% Dimesions of matrix A
dimA = size(A);

% Transpose of matrix A
At = A';

% Logical indexing
% Get truth values for all elements of A to be greater than 3
tt = A > 3 ;

% Slicing matrices
% Get the 2x3 submatrices from A
A1 =  A(1:2,:);
A2 =  A(2:3,:);
A3 =  [A(3,:);A(1,:)];
%% Matlab documentation
% Using doc `name` and help `name` for help
% Try using the above commands to know more about any of the commands used
% Read Matlab documentation to find out more examples and arguments,
% overloaded functions 
% Read the documentation to create a matrix of size (5,7) with random
% elements between [-12, 44]
tic
C3 = -12+(44+12).*rand(5,7);
C3 = randi([-12,44],[5,7]);
% Reshape matrix A into a vector
av = reshape(A,[],1);
av = reshape(A,9,1);
% What happened to the order of the elements?
% Repeat the same for A' and observe the order
av2 = reshape(A',[],1);

% Append the vector to itself
avav = [av;av];

% Now reshape the vector into a 6x3 matrix
Av = reshape(avav,[6,3]);
% What happened to the order of the elements?

%% Utilities
% Using clear all and clc to keep the console and workspace clean
% Using tic and toc to measure execution time of a piece of code
time = toc;

%% Plotting
% Now read the documentation for plotting and plot a tan curve for angles
% between [-pi/2, pi/2] with a dashed blue line
x = -pi/2+0.01:0.01:pi/2-0.01;
figure;
plt = plot(x,tan(x));

% On the same plot as above plot a contour plot for the function
% (x-1)^2+(y-3)^2 for x, y values between [-3,3], see `contour`, `hold
% on` and `meshgrid` functions
x = -3:0.01:3;
y = -3:0.01:3;
[X,Y] = meshgrid(x,y) ;
hold on; 


% Without elementwise mult
% Z = (X-1)^2+(Y-3)^2;
% cnt_plot = contour(X,Y,Z);

% What happens to the plot? How would you rectify the scaling? Try changing
% the order of the plots

%With elementwise mult
Z = (X-1).^2+(Y-3).^2;
cnt_plot = contour(X,Y,Z);

% How does the behavior change with and without the elementwise operator
% and why? Hint: Try expanding the former and see what it results in. 

% Tryout `contour3`, `plot3`, `surface` commands for the same