clear all
clc

% Mechanical design variables
m = 5; % kg
l = 0.5; % m
g = 9.8; % m/s^2
b = 1; % N/(rad/s)
% x1 = theta
% x2 = dtheta
u = 0;
% Phase portrait for the free system
dx1dt = @(x1,x2)x2 ;
dx2dt = @(x1,x2)(u-(m*g*l*sin(x1)+b*x2)) / (m * l^2);


% See the documentation for the quiver function and plot it for 
% the given range of plot
% What does this give us?
[x1,x2] = meshgrid(-6:0.5:6,-6:0.5:6);
dX1 = dx1dt(x1,x2);
dX2 = dx2dt(x1,x2);
quiver(x1,x2,dX1,dX2);
hold on

%% Plot the motion of the pendulum
% Create a function called `pendulum_dynamics` with natural dynamics of the pendulum
% Choose an inital position and a velocity and integrate the pendulum
% dynamics
x0 = [1;1];
% Final time until which you want the system to be simulated
t_end = 15;
tspan = [0 t_end];
% Use ode-45 to integrate the system
[t,z] = ode45(@pendulum_dynamics,tspan,x0);
% Plot the trajectory obtained as z over the the quiver plot
hold on
plot(z(:,1),z(:,2),'-');



%% Formulate a control law proportional to the error between the angle 
% and the reference angle

% Declare a proportional gain component
Kp = 30;
% Reference angle and velocity 
ref_z = [pi;0];
% Write the control law using only Kp
u = Kp * (ref_z(1) - x1);

%% Now the phase portrait of the controlled system
% System dynamics of the controlled system
dx1dt = @(x1,x2)x2;
dx2dt = @(x1,x2)(u-(m*g*l*sin(x1)+b*x2)) / (m * l^2);

% Plot phase-space using the controlled system
% Range of plot
[x1,x2] = meshgrid(-6:0.5:6,-6:0.5:6);

dX1 = dx1dt(x1,x2);
dX2 = dx2dt(x1,x2);
quiver(x1,x2,dX1,dX2);
hold on

%% Control the inverted pendulum
% Write a function called `controlled_pendulum_dynamics` with controlled dynamics 
% and the chosen control law 
% Lets use a zero order hold to demonstrate an actual simulation
ref_z = [pi,0];
z0 = [0,0];
t_end = 5;
Kp = 1;
Kd = 0;
Ki = 0;
z_current = z0;
t_step = 1e-3;
z_vals = [z0];
for i=0:t_step:20
    u = Kp*(ref_z(1) - z_current(1));
    % Let the system run with the computed control (zero order hold)
    [t,z] = ode45(@(t,z)controlled_pendulum_dynamics(t,z,u));
    % Get the sensor reading for the joint position
    z_current = z(end,:);
    z_vals(end+1,:) = z_current;
    % Repeat
end
% 
% %% Overlay control trajectory onto the phase portrait
% 
% 
% %% Exercises to explore
% % Q1. What happens when the damping is removed? 
% % Q2. How do we extend the controller to PD?