function u_target = Cruise_control(psi, X, Y, u, cruise_params, prev_idx, road)
% CRUISE_CONTROL  Target speed based on look-ahead curvature.
%
%   Structurally identical to the Steering MATLAB Function block:
%     - Same sliding-window position search (Part 1)
%     - Same forward walk to build lookahead window (Part 2)
%   Then differs in what it does with the preview:
%     - Scans every A-B-C triplet inside the window for curvature (Part 3)
%     - Outputs  u_target = sqrt(ay_max * R_min)             (Part 4)
%
%   INPUTS
%     psi           – yaw angle [rad]  (port compatibility only)
%     X, Y          – vehicle CG position [m]
%     u             – current speed [m/s]
%     cruise_params – struct: .Tp, .ay_max (default 7.5), .u_min (default 2)
%     prev_idx      – shared index fed from the Steering next_idx output
%     road          – Nx2 [X Y] centreline [m]
%
%   OUTPUT
%     u_target      – target speed [m/s]  →  "set speed" in Simulink

    % ---- Mandatory output init (MATLAB Coder requirement) ----
    u_target = cruise_params.u_min;   % safe default

    % ---- Parameters ----
    Tp     = cruise_params.Tp;
    ay_max = 7.5;
    if isfield(cruise_params, 'ay_max'), ay_max = cruise_params.ay_max; end
    u_min = 2.0;
    if isfield(cruise_params, 'u_min'),  u_min  = cruise_params.u_min;  end
    u_max = 30.0;  % default ~108 km/h
    if isfield(cruise_params, 'u_max'),  u_max  = cruise_params.u_max;  end

    % ---------------------------------------------------------------- %
    %  1. Find current segment – identical to Steering MLFB              %
    % ---------------------------------------------------------------- %
    search_window = 100;
    N = size(road, 1);

    idx      = prev_idx;
    min_dist = inf;

    start_k = prev_idx;
    end_k   = min(prev_idx + search_window, N - 1);

    % End simulation after reaching the end of the track
    for i = start_k : end_k
        p1 = road(i,   :);
        p2 = road(i+1, :);

        t_vec   = p2 - p1;
        seg_len = norm(t_vec);
        if seg_len < 1e-6, continue; end
        t_hat = t_vec / seg_len;

        v_vec    = [X, Y] - p1;
        proj_len = dot(v_vec, t_hat);

        % Calculate distance to segment
        curr_d = norm(v_vec - proj_len * t_hat);

        % Add penalty if outside segment to favour the correct segment
        if proj_len < 0 || proj_len > seg_len
            curr_d = curr_d + 10.0;
        end

        if curr_d < min_dist
            min_dist = curr_d;
            idx      = i;
        end
    end

    next_idx = idx;   % current best segment

    % ---------------------------------------------------------------- %
    %  2. Walk forward – identical to Steering MLFB (previewDist)       %
    % ---------------------------------------------------------------- %
    previewDist = max(5.0, u * Tp);   % same formula as steering

    travelled = 0;
    j = next_idx;
    while j < N - 1 && travelled < previewDist
        ds        = norm(road(j+1,:) - road(j,:));
        travelled = travelled + ds;
        j         = j + 1;
    end
    % j is now the far end of the lookahead window

    % ---------------------------------------------------------------- %
    %  3. Curvature scan – every A-B-C triplet in the window            %
    % ---------------------------------------------------------------- %
    R_min = inf;
    k     = next_idx;

    while k <= j - 2
        A = road(k,   :);
        B = road(k+1, :);
        C = road(k+2, :);

        r1     = B - A;
        r2     = C - B;
        mag_r1 = norm(r1);
        mag_r2 = norm(r2);

        if mag_r1 < 1e-6 || mag_r2 < 1e-6
            k = k + 1;
            continue;
        end

        delta_S     = (mag_r1 + mag_r2) / 2.0;
        cp_z        = r1(1)*r2(2) - r1(2)*r2(1);
        sin_dtheta  = max(-1.0, min(1.0, cp_z / (mag_r1 * mag_r2)));
        delta_theta = asin(sin_dtheta);
        kappa       = abs(delta_theta) / delta_S;

        if kappa > 1e-9
            R_local = 1.0 / kappa;
            if R_local < R_min
                R_min = R_local;
            end
        end

        k = k + 1;
    end

    % Straight ahead – cap at u_max so PI controller doesn't apply max torque forever
    if isinf(R_min)
        R_min = (u_max^2) / ay_max;  % back-calculate R that gives exactly u_max
    end

    % ---------------------------------------------------------------- %
    %  4. Target speed  (v = sqrt(ay_max * R_min))                     %
    % ---------------------------------------------------------------- %
    u_target = sqrt(ay_max * R_min);
    u_target = max(u_target, u_min);
    u_target = min(u_target, u_max);  % hard cap

end
