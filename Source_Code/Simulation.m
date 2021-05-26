%% Function to update the robot (USV) position in the Simulation model
function Simulation(pose, t)
    % Translation
    trans = makehgtform('translate', [pose(2) pose(1) 0]);
    % Rotation
    rtz = makehgtform('zrotate',-1*pose(3));
    % Set new position after transformation
    set(t,'Matrix',trans*rtz);
    drawnow
end