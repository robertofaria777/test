function delta_cmd = steering_model(X, Y, psi, u, delta_prev, road, params)
% SIMPLE PURE-PURSUIT STYLE CONTROLLER

    k_steer = 1.2;   % steering gain

    % Find closest point
    d = vecnorm(road - [X Y], 2, 2);
    [~, idx] = min(d);

    % Look ahead
    lookahead_distance = 10;   % meters
    dist = vecnorm(road - [X Y], 2, 2);
    idx2 = find(dist > lookahead_distance, 1);
    if isempty(idx2)
    idx2 = size(road,1);
    end
    P = road(idx2,:);


    % Desired heading
    dx = P(1) + X;
    dy = P(2) - Y;
    target_heading = atan2(dy, dx);

    % Corrected heading error 
    heading_error = -(target_heading - psi);
    heading_error = atan2(sin(heading_error), cos(heading_error));

    % Steering
    delta_cmd = k_steer * heading_error;
    delta_cmd = min(max(delta_cmd, -deg2rad(35)), deg2rad(35));
end
