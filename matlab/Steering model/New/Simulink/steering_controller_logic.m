function [delta_cmd, next_idx] = steering_controller_logic(X, Y, psi, u, delta_prev, road, Tp, Klat, L, Kug, prev_idx)
% STEERING_CONTROLLER_LOGIC
% Simulink MATLAB Function Block wrapper of driver_model_paper logic.
%
% This function is a faithful, Simulink-compatible reimplementation of
% the preview driver model (Eq 2, 4, 5, 9, 11 of the paper).
%
% Inputs:
%   X, Y       : Vehicle global position (m) - from 6DOF integrator
%   psi        : Vehicle yaw angle (rad) - from 6DOF integrator
%   u          : Longitudinal speed (m/s) - from 6DOF state
%   delta_prev : Previous step steering angle (rad) - via Unit Delay
%   road       : [N x 2] track centreline [x, y] - from Base Workspace
%   Tp         : Preview time (s)
%   Klat       : Lateral gain
%   L          : Wheelbase (m) - should match init6dof_v4 value (2.83 m)
%   Kug        : Understeer gradient (0 = neutral steer)
%   prev_idx   : Track index from previous step - via Unit Delay (IC=1)
%
% Outputs:
%   delta_cmd  : Commanded front steering angle (rad) - to vehicle input
%   next_idx   : Updated track index - fed back via Unit Delay

    N = size(road, 1);

    % -------------------------------------------------------
    % 1. Find closest road segment to current vehicle position
    % -------------------------------------------------------
    search_window = 100;
    idx      = prev_idx;
    min_dist = inf;
    start_k  = prev_idx;
    end_k    = min(prev_idx + search_window, N - 1);

    for i = start_k : end_k
        p1 = road(i,   :);
        p2 = road(i+1, :);

        t_vec   = p2 - p1;
        seg_len = norm(t_vec);
        if seg_len < 1e-6, continue; end
        t_hat = t_vec / seg_len;

        v_vec    = [X, Y] - p1;
        proj_len = dot(v_vec, t_hat);

        curr_d = norm(v_vec - proj_len * t_hat);

        if proj_len < 0 || proj_len > seg_len
            curr_d = curr_d + 10.0; % Penalize out-of-segment projections
        end

        if curr_d < min_dist
            min_dist = curr_d;
            idx      = i;
        end
    end

    next_idx = idx; % double — safe for Simulink Unit Delay

    % -------------------------------------------------------
    % 2. Calculate kinematic preview point P (Eq 2, 4, 5)
    % -------------------------------------------------------
    if abs(delta_prev) < 1e-5
        R = inf;
    else
        R = (L + Kug * u^2) / delta_prev;
    end

    tG = [cos(psi), sin(psi)];   % vehicle forward tangent
    nG = [-sin(psi), cos(psi)];  % vehicle left normal

    if isinf(R)
        P = [X, Y] + u * Tp * tG;
    else
        O     = [X, Y] + R * nG;
        theta = (u * Tp) / R;
        v_vec_circ = -R * nG;

        cos_t = cos(theta);
        sin_t = sin(theta);
        v_rot = [v_vec_circ(1)*cos_t - v_vec_circ(2)*sin_t, ...
                 v_vec_circ(1)*sin_t + v_vec_circ(2)*cos_t];
        P = O + v_rot;
    end

    % -------------------------------------------------------
    % 3. Lateral error at preview point (Eq 9)
    % -------------------------------------------------------
    best_d      = inf;
    found_valid = false;
    search_fwd  = 200;

    for k = next_idx : min(next_idx + search_fwd, N - 1)
        curr_S  = road(k,   :);
        next_S  = road(k+1, :);

        seg_vec = next_S - curr_S;
        s_len   = norm(seg_vec);
        if s_len < 1e-6, continue; end
        t_hat_L   = seg_vec / s_len;

        p_vec = P - curr_S;
        proj  = dot(p_vec, t_hat_L);

        if proj >= -1.0 && proj <= s_len + 1.0  % 1 m slack
            n_L_local = [-t_hat_L(2), t_hat_L(1)]; % left normal
            dist = dot(p_vec, n_L_local);

            if abs(dist) < abs(best_d)
                best_d      = dist;
                found_valid = true;
            end
        end
    end

    if ~found_valid
        % Fallback: nearest tangent projection at next_idx
        curr_S  = road(next_idx, :);
        next_S  = road(min(next_idx + 1, N), :);
        seg_vec = next_S - curr_S;
        t_hat_L   = seg_vec / norm(seg_vec);
        n_L_local = [-t_hat_L(2), t_hat_L(1)];
        best_d    = dot(P - curr_S, n_L_local);
    end

    d_lat = best_d;

    % -------------------------------------------------------
    % 4. Steering control law (Eq 11)
    %    delta(k+1) = delta(k) - Klat * d_lat
    % -------------------------------------------------------
    delta_cmd = delta_prev - Klat * d_lat;

    % 5. Saturation (+/- 35 deg)
    delta_max = 0.6109; % deg2rad(35)
    delta_cmd = max(-delta_max, min(delta_cmd, delta_max));

end
