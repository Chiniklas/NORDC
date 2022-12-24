%% Constraints on the states
% Current approximation of dynamics is by Trapezoid Quadrature
function  [cineq, ceq] = const(w,init)
% Inequality constraints
cineq = [];

% Extract N, n, nc, x0, xf and h from init

x0 = init.x0;
xf = init.xf;
N = init.N;
n = init.n;
nc = init.nc; 
h = init.h; 

%% Equality constraints (do not modify)
%Constraining intial conditions
ceq(1:n) = w(nc*N+n+1:nc*N+n*2)-x0-h/2*(evaldyn([x0;w(1:nc)])+evaldyn([w(nc*N+n+1:nc*N+n*2);w(nc+1:2*nc)]));
%Collocation constraints
for i=1:N-1
    ceq(end+1:end+n)=w(nc*N+1+n*i:nc*N+n*(i+1))-w(nc*N+1+n*(i-1):nc*N+n*i)-h/2*(evaldyn([w(nc*N+1+n*i:nc*N+n*(i+1));w(nc*i+1:nc*(i+1))])+evaldyn([w(nc*N+1+n*(i-1):nc*N+n*i);w(nc*(i-1)+1:nc*i)]));
end
%Constraining the end state
ceq(end+1:end+n) = xf-w(end-2*n+1:end-n)-h/2*(evaldyn([xf;w(nc*(N-1)+1:nc*N)])+evaldyn([w(end-2*n+1:end-n);w(nc*(N-2)+1:nc*(N-1))]));
% Making the final control to be zero
ceq(end+1:end+nc) = w(nc*(N-1)+1:nc*N);

% States
x = [x0,reshape(w(nc*N+1:end),n,N)];
assignin('base', 'x', x)