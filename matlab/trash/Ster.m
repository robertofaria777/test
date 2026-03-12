% Inputs:
%   Xg, Yg      = global position of vehicle CG
%   psi         = vehicle yaw angle (rad)
%   u           = longitudinal speed (m/s)
%   delta_prev  = previous steering angle (rad)
%   road       = [Nx2] array of track points [X Y]
%   params      = structure of driver parameters
%
% Outputs:
%   delta_cmd   = steering angle command (rad)

function delta_cmd = steering_model(Xg, Yg, psi, u, delta_prev, road, params)

    % -------------------------
    % Driver parameters
    % -------------------------
    Tp   = params.Tp;     % preview time (s)
    Klat = params.Klat;   % steering gain
    Kug  = params.Kug;    % understeer gradient (deg/g)
    L    = params.L;      % wheelbase (m)
    g    = 9.81;

    % Convert understeer gradient deg/g → rad/(m/s^2)
    Kug_rad = deg2rad(Kug) / g;

    % -------------------------
    % 1. Find preview point along road
    % -------------------------
    % distance to preview (s * m/s)
    previewDistance = max(0.1, Tp * u);

    % find nearest point on the road
    d = vecnorm(road - [Xg Yg], 2, 2);
    [~, idx] = min(d);

    % walk forward until preview distance is reached
    traveled = 0;
    i = idx;
    while traveled < previewDistance && i < size(road,1)
        stepd = norm(road(i+1,:) - road(i,:));
        traveled = traveled + stepd;
        i = i + 1;
    end

    P = road(i,:);    % preview point coordinates

    % -------------------------
    % 2. Compute lateral deviation at preview point
    % -------------------------
    % Global unit vectors based on heading ψ
    tG = [cos(psi), sin(psi)];     % forward direction
    nG = [-sin(psi), cos(psi)];    % left normal

    GP = P - [Xg Yg];
    d_lat = dot(GP, nG);  % signed lateral error

    % -------------------------
    % 3. Compute steady-state curvature due to steering
    % -------------------------
    % AVEC: R = u^2 / (g*(Kus*u + δ))
    % Rearranged for δ(k+1)
    %
    %      δ = δ_prev - Klat * d_lat
    %
    % (Eq. 11 of AVEC paper)
    % :contentReference[oaicite:2]{index=2}
    % -------------------------

    delta_cmd = delta_prev - Klat * d_lat;


end
