function [W] = workspace_area(dv, p)
%WORKSPACE_AREA Area of the workspace
r = dv(1)+dv(2);
d = dv(3);
% Design parameters
a = p(1);
b = p(2);

h = sqrt(r^2 - (d/2)^2);
% Workspace based on the conditions
% When there is no intersection
if r < d/2
    W = 0;
end

% When there is intersection
if r >= d/2
    W = pi*r^2*(acos(d/(2*r))/pi) - d/2*h;
    W = W - 1/8* pi * a * b;
end

