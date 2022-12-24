function [z] = controlled_pendulum_dynamics(t,x,u)
    m = 5; % kg
    l = 0.5; % m
    g = 10; % m/s^2
    b = 1; % N/(rad/s)

    dx1 = x(2) ;
    dx2 = (u-(m*g*l*sin(x(1))+b*x(2))) / (m * l^2);
    
    z = [dx1;dx2];
end