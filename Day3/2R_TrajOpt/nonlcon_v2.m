function [c, ceq] = nonlcon(w, init)
% Set any non-linear inequality constraints

% Set any linear equality constraints, i.e., 
% enforce dynamics at all knot points
% Get required values from init
t0 = init.t0 ;
tf = init.tf ;
l1 = init.l1 ;
l2 = init.l2 ;
m1 = init.m1 ;
m2 = init.m2 ;
me = init.me ;
N = init.N;
h = init.h;
x0 = init.x0;
xf = init.xf;
% Resolve all the states and controls into an index sliced matrix
% Output Nx2 matrix where each row is one discretisation of time
u = w(1:200);
reshape(u,[],2);
% Output Nx4 matrix where each row is one discretisation of time
z = w(201:end); 
reshape(z,[],4);
% Enforce the dynamics constraint at each of the knot points using the
% 2R-robot dynamics given in the file `dyn2R`
c = [];

ceq = zeros(N,4);
% Equality constraints at t0 to ensure the initial state63
ceq(1,:) = z(1,:)-x0;
% Dynamics constraint at all intermediate steps
for i = 2 : N-1
    % Get dynamics function values at k+1 and k
    ck = dyn2R(reshape(z(i,:),[],1),reshape(u(i,:),[],1),init);
    ck_1 = dyn2R(reshape(z(i+1,:),[],1),reshape(u(i+1,:),[],1),init);
    % Now add the constraints
    ceq(i,:) = z(i+1,:)-z(i,:)-h.*(ck + ck_1)'/2;
end
% Equality constraints at tf to ensure the final state
ceq(N,:) = z(N,:)-xf;
%ceq

ceq = reshape(ceq,[],1);
%% Constraint more last states to not just reach but reach and 
% hold the state
end