function signal = cable_balljoint_collision(A,B,init)

% Init values for ballcenter, ballradius, nTendons
ballcenter = init.ballcenter;
ballradius = init.ballradius;
nTendons = init.nTendons;
signal = 1;
% % Check collision for each tendon
for i=1:nTendons
    % Get vectors corresponding to cable direction
    v = norm(A(:,i)-B(:,i));
    % Get closest distance between the cable and the ball
    dist = norm(cross(A(:,i)-ballcenter,B(:,i)-ballcenter))/v;
    
    if dist < ballradius
        signal = 0;
        break
    end
end

end
