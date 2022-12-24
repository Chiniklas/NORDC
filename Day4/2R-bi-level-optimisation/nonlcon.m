function [c, ceq] = nonlcon(w, init)
% Set any non-linear inequality constraints

% Set any linear equality constraints, i.e., 
% enforce dynamics at all knot points
% Get required values from init
n = init.n;
nc = init.nc;
N = init.N;
h = init.h;
x0 = init.x0;
xf = init.xf;
% Resolve all the states and controls into an index sliced matrix
% Output Nx2 matrix where each row is one discretisation of time
u = w(1:nc*N);
u = reshape(u,[nc,N]);
% u = u';
% Output Nx4 matrix where each row is one discretisation of time
z = w(nc*N+1:end); 
z = reshape(z,[n,N]);
% z = z';
% Enforce the dynamics constraint at each of the knot points using the
% 2R-robot dynamics given in the file `dyn2R`
c = [];

ceq = zeros(n,N+1);
% Equality constraints at t0 to ensure the initial state63
ceq(:,1) = z(:,1)-x0;
% Dynamics constraint at all intermediate steps
for i = 1 : N-1
    % Get dynamics function values at k+1 and k
    ck = dyn2R(z(:,i),u(:,i),init);
    ck_1 = dyn2R(z(:,i+1),u(:,i+1),init);
    % Now add the constraints
    ceq(:,i+1) = z(:,i+1)-z(:,i)-h.*(ck + ck_1)/2;
end
% Equality constraints at tf to ensure the final state
ceq(:,N+1) = z(:,N)-xf;
%ceq

ceq = reshape(ceq,[],1);
%% Constraint more last states to not just reach but reach and 
% hold the state
end