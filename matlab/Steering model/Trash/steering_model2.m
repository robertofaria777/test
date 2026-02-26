function delta_cmd = steering_model(X, Y, psi, u, delta_prev, road, params)
    g = 9.81;

    Tp   = 100; %params.Tp; preview time
    Klat = 0.1; %params.Klat; lateral gain
    Kug  = 0;   %params.Kug; understeer gradient (deg/g)
    L    = 2.6; %params.L;  wheelbase

    % Convert Kug to rad/(m/s^2)
    Kug_rad = deg2rad(Kug) / g;

    % 1. Find closest road point
    d = vecnorm(road - [X Y], 2, 2);
    [~, idx] = min(d);

    % If we are at end of road
   if idx >= size(road,1)
    idx = size(road,1);
end

    % 2. Choose preview point by distance (10 meters)
    lookahead = 10; % meters
    distToAll = vecnorm(road - [X Y], 2, 2);
    idx2 = find(distToAll > lookahead, 1);

    if isempty(idx2)
        idx2 = size(road,1);
    end
    Ptrack = road(idx2,:);

    % 3. Steady-state curvature model
    delta = delta_prev;

    denom = (delta + Kug_rad*u);
    if abs(denom) < 1e-6
        denom = sign(denom)*1e-6;
    end

    R = u^2 / (g * denom);        % steady-state turning radius
    theta = (u * Tp) / R;         % angle swept over preview time

    % 4. Vehicle unit vectors
    tG = [cos(psi), sin(psi)];      % forward direction
    nG = [-sin(psi), cos(psi)];     % left direction

    % Circle center O (positive R -> turning left)
    O = [X Y] + R * nG;

    % Vector from center to vehicle
    r0 = [X Y] - O;

    % Rotation matrix
    rot = [cos(theta) -sin(theta); 
           sin(theta)  cos(theta)];

    % Rotate vector around the circle to get predicted preview position
    r_pred = (rot * r0')';
    Ppred = O + r_pred;
    
    % 5. Lateral error at predicted preview point
    lat_error = dot(Ppred - [X Y], nG);
    
    % 6. Steering correction
    delta_cmd = delta_prev - Klat * lat_error;
    
    % 7. Steering saturation
    delta_max = deg2rad(35);
    delta_cmd = min(max(delta_cmd, -delta_max), delta_max);

end
