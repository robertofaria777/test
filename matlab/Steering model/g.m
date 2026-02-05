function delta_cmd = steering_model(X, Y, psi, u, delta_prev, road, params)
% AVEC-BASED STEERING CONTROLLER
% Implements the core lateral control loop from the AVEC paper:
% 1. Predict vehicle position (Ppred) based on current steer angle (delta)
% 2. Calculate the lateral deviation (d_L) of Ppred from the road segment.
% 3. Correct the steer angle: delta_cmd = delta_prev - Klat * d_L
%
% Based on: "A Simple Realistic Driver Model" by Matthew C Best, AVEC 12.

    % Global constant
    g = 9.81;

    % Driver and Vehicle Parameters (Nominal values from AVEC Table 1 for M)
    Tp   = 0.5;      % Preview Time (s)
    Klat = 0.1;      % Lateral Gain (rad/m)
    Kug  = 1;        % Understeer gradient (deg/g)
    L    = 2.6 %params.L; % Wheelbase (m)

    % Convert Kug from deg/g to rad/(m/s^2) for consistent units
    Kug_rad = deg2rad(Kug) / g;

    %% ----------------------------------------------------------
    % 1. Find closest road point to vehicle position
    %% ----------------------------------------------------------
    % Find the point on the road closest to the current vehicle position [X Y]
    d = vecnorm(road - [X Y], 2, 2);
    [~, idx] = min(d);

    % Define the road segment (S_L to E_L)
    if idx >= size(road,1)
        delta_cmd = 0; % End of road, stop steering
        return
    end
    
    % Ensure idx is not the last point to form a segment
    if idx == size(road, 1)
        idx = size(road, 1) - 1;
    end
    
    S_L = road(idx,:);      % Start of Segment (closest point)
    E_L = road(idx+1,:);    % End of Segment

    %% ----------------------------------------------------------
    % 2. Predict Preview Position (Ppred)
    %% ----------------------------------------------------------
    
    % Calculate the steady-state turning radius R (Eq. 2 from paper, adapted)
    % R = (L + (Kug*u^2)/g) / delta_prev
    numerator_R = L + Kug_rad * u^2;
    
    % Avoid division by zero when delta_prev is near zero
    if abs(delta_prev) < 1e-4
        % Set a very large radius (straight line) based on sign
        delta_sign = sign(delta_prev);
        if delta_sign == 0, delta_sign = 1; end
        R = numerator_R / (delta_sign * 1e-4);
    else
        R = numerator_R / delta_prev;
    end
    
    % Angle swept over preview time (Eq. 4)
    theta = (u * Tp) / R;

    % Vehicle unit vectors
    % tG: forward direction, nG: left direction
    tG = [cos(psi), sin(psi)];
    nG = [-sin(psi), cos(psi)];

    % Circle center O (based on R and nG)
    O = [X Y] + R * nG;

    % Vector from center to vehicle G
    r0 = [X Y] - O;

    % Rotation matrix for angle theta
    rot = [cos(theta) -sin(theta);  
           sin(theta)  cos(theta)];

    % Rotate vector r0 to get predicted position Ppred (Eq. 5)
    r_pred = (rot * r0')';
    Ppred = O + r_pred;

    %% ----------------------------------------------------------
    % 3. Lateral error at predicted preview point (d_L)
    %    Deviation of Ppred from the road segment (Eq. 8)
    %% ----------------------------------------------------------
    
    % Road tangent vector (r_L) and its unit vector
    r_L = E_L - S_L;
    r_L_unit = r_L / norm(r_L);
    
    % Road normal vector (n_L), 90 deg counter-clockwise from tangent
    n_L = [-r_L_unit(2), r_L_unit(1)];

    % Lateral error d_L = (Ppred - S_L) . n_L 
    % Positive means Ppred is to the left of the road segment
    lat_error_avec = dot(Ppred - S_L, n_L);

    %% ----------------------------------------------------------
    % 4. AVEC steering correction (Eq. 11)
    %% ----------------------------------------------------------
    % delta_cmd = delta_prev - Klat * d_L
    delta_cmd = delta_prev - Klat * lat_error_avec;

    %% ----------------------------------------------------------
    % 5. Steering saturation
    %% ----------------------------------------------------------
    delta_max = deg2rad(35); % Max steering angle (35 degrees)
    delta_cmd = min(max(delta_cmd, -delta_max), delta_max);

end