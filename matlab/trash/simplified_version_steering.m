function delta = simple_path_following(X, Y, psi, road)


L = 2.6;          % wheelbase (m)
lookahead = 10;   % 10 m lookahead distance
k_steer = 1.5;    % steering gain

% --------------------------------------------------
% 1. Find closest point on the road
% --------------------------------------------------
d = vecnorm(road - [X Y], 2, 2);
[~, idx] = min(d);

% --------------------------------------------------
% 2. Choose a point ahead on the road
% --------------------------------------------------
idx2 = min(idx + 5, size(road,1));   % look ahead 5 points
P = road(idx2,:);                    % preview target

% --------------------------------------------------
% 3. Compute target direction
% --------------------------------------------------
dx = P(1) - X;
dy = P(2) - Y;
target_heading = atan2(dy, dx);

% --------------------------------------------------
% 4. Heading error
% --------------------------------------------------
heading_error = target_heading - psi;
heading_error = atan2(sin(heading_error), cos(heading_error)); % wrap -pi..pi

% --------------------------------------------------
% 5. Steering command
% --------------------------------------------------
delta = k_steer * heading_error;

% Limit steering
delta = max(min(delta, deg2rad(35)), -deg2rad(35));
end
