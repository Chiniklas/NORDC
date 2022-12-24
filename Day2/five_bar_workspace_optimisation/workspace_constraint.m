function [c, ceq] = workspace_constraint(dv,p)
% Note: Since these are linear constraints, you can directly use the Aeq,
% beq and A, B matrices in fmincon

L = ;
b = ;
d = ;

% Constraints on `b` and `d`
c = ;
% Constraints on the total length
ceq = ;
end