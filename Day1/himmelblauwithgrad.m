function [f,grad] = himmelblauwithgrad(x)
% Calculate objective f
f = (x(1).^2 + x(2) - 11).^2 + (x(1) + x(2).^2 - 7).^2;

if nargout > 1 % gradient required
    grad = [4.*x(1).*(x(1).^2 + x(2) - 11) + 2.*(x(1) + x(2).^2 - 7);
        2.*(x(1).^2 + x(2) - 11) + 4.*x(2).*(x(1) + x(2).^2 - 7)];
end
end