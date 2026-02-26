%function [delta_cmd, next_idx, debug_info] = driver_model_paper(X, Y, psi, u, delta_prev, road, params, prev_idx)
    % DRIVER_MODEL_PAPER - Steering control
    %
    % Inputs:
    %   X, Y       : Vehicle Global Position
    %   psi        : Vehicle Yaw Angle (rad)
    %   u          : Longitudinal Speed (m/s)
    %   delta_prev : Previous steering angle (rad) (Used for continuity)
    %   road       : [x, y] coordinates of track centerline
    %   params     : Struct with parameters (Tp, Klat, Klong, etc.)
    %   prev_idx   : Index of the track segment from the previous step (optimization)
    %
    % Outputs:
    %   delta_cmd  : Commanded steering angle (rad)
    %   next_idx   : Updated index for the current track segment
    %   debug_info : Struct with P, d_lat, of the preview point
    
    % Extract Parameters
    Tp = params.Tp;       % Preview time (s)
    Klat = params.Klat;   % Lateral gain
    if isfield(params, 'L'), L = params.L; else, L = 2.5; end
    if isfield(params, 'Kug'), Kug = params.Kug; else, Kug = 0; end
    
    % Find Current Position S_G on Track
    % Search window from previous index
    search_window = 100; 
    N = size(road, 1);
    
    idx = prev_idx;
    min_dist = inf;
    
    start_k = prev_idx;
    end_k = min(prev_idx + search_window, N-1);
    
    % End simulation after reaching the end of the track
    
    for i = start_k : end_k
        p1 = road(i, :);
        p2 = road(i+1, :);
        
        t_vec = p2 - p1;
        seg_len = norm(t_vec);
        if seg_len < 1e-6, continue; end
        t_hat = t_vec / seg_len;
        
        v_vec = [X, Y] - p1;
        proj_len = dot(v_vec, t_hat);
        
        % Calculate distance to segment
        curr_d = norm(v_vec - proj_len * t_hat);
        
        % Add penalty if outside segment to favor the road segment
        if proj_len < 0 || proj_len > seg_len
            curr_d = curr_d + 10.0; 
        end
        
        if curr_d < min_dist
             min_dist = curr_d;
             idx = i;
        end
    end
    
    next_idx = idx; 
    
    % Calculate Kinematic Preview Point P 
    % Standard Steady State Radius R (Eq 2)
    if abs(delta_prev) < 1e-5
        R = inf;
    else
        R = (L + Kug * u^2) / delta_prev;
    end
    
    % Heading/Normal vectors
    tG = [cos(psi), sin(psi)];
    nG = [-sin(psi), cos(psi)]; % Normal to Left
    
    if isinf(R)
        % Straight line projection
        P = [X, Y] + u * Tp * tG;
    else
        % Circular projection (Eq 5)
        % Center of curvature O = G + R * nG
        O = [X, Y] + R * nG;
        
        % Angle traversed theta (Eq 4)
        theta = (u * Tp) / R;
        
        % Vector from O to G is -R*nG. Rotate it by theta.
        v = -R * nG;
        
        cos_t = cos(theta);
        sin_t = sin(theta);
        
        % 2D Rotation
        v_rot = [v(1)*cos_t - v(2)*sin_t, v(1)*sin_t + v(2)*cos_t];
        
        P = O + v_rot;
    end
    
    % Lateral Error Calculation
    % Find segment P falls on.
    % Search forward from 'next_idx' since P is ahead.
    
    best_d = inf;
    best_idx = next_idx;
    found_valid = false;
    
    search_fwd = 200; % Larger window for lookahead
    
    for k = next_idx : min(next_idx + search_fwd, N-1)
        curr_S = road(k, :);
        next_S = road(k+1, :);
        
        seg_vec = next_S - curr_S;
        s_len = norm(seg_vec);
        if s_len < 1e-6, continue; end
        t_hat_L = seg_vec / s_len;
        
        p_vec = P - curr_S;
        proj = dot(p_vec, t_hat_L);
        
        % Check if valid segment (Eq 9)
        if proj >= -1.0 && proj <= s_len + 1.0 % Allow 1m slack
             % Valid segment
             n_L_local = [-t_hat_L(2), t_hat_L(1)]; % Left Normal
             dist = dot(p_vec, n_L_local);
             
             % Find smallest valid d (closest segment involved)
             if abs(dist) < abs(best_d)
                 best_d = dist;
                 best_idx = k;
                 found_valid = true;
             end
        end
    end
    
    if ~found_valid
        % Fallback: P is likely far outside track or off the end
        % Use closest point logic from next_idx
        p_start = road(next_idx, :);
        dist_direct = norm(P - p_start);
                
        % Projection onto the 'next_idx' tangent extended.
        curr_S = road(next_idx, :);
        next_S = road(next_idx+1, :);
        seg_vec = next_S - curr_S;
        t_hat_L = seg_vec / norm(seg_vec);
        n_L_local = [-t_hat_L(2), t_hat_L(1)];
        best_d = dot(P - curr_S, n_L_local);
    end
    
    d_lat = best_d;
    
    debug_info.P = P;
    debug_info.d_lat = d_lat;
    
    %Steering Update (Eq 11) 
    % delta(k+1) = delta(k) - Klat * d
    delta_cmd = delta_prev - Klat * d_lat;
    
    % Saturation
    delta_max = deg2rad(35);
    delta_cmd = max(-delta_max, min(delta_cmd, delta_max));
    
end
    % Extract Parameters
    Tp = params.Tp;       % Preview time (s)
    Klat = params.Klat;   % Lateral gain
    if isfield(params, 'L'), L = params.L; else, L = 2.5; end
    if isfield(params, 'Kug'), Kug = params.Kug; else, Kug = 0; end
    
    % Find Current Position S_G on Track
    % Search window from previous index
    search_window = 100; 
    N = size(road, 1);
    
    idx = prev_idx;
    min_dist = inf;
    
    start_k = prev_idx;
    end_k = min(prev_idx + search_window, N-1);
    
    % End simulation after reaching the end of the track
    
    for i = start_k : end_k
        p1 = road(i, :);
        p2 = road(i+1, :);
        
        t_vec = p2 - p1;
        seg_len = norm(t_vec);
        if seg_len < 1e-6, continue; end
        t_hat = t_vec / seg_len;
        
        v_vec = [X, Y] - p1;
        proj_len = dot(v_vec, t_hat);
        
        % Calculate distance to segment
        curr_d = norm(v_vec - proj_len * t_hat);
        
        % Add penalty if outside segment to favor the road segment
        if proj_len < 0 || proj_len > seg_len
            curr_d = curr_d + 10.0; 
        end
        
        if curr_d < min_dist
             min_dist = curr_d;
             idx = i;
        end
    end
    
    next_idx = idx; 
    
    % Calculate Kinematic Preview Point P 
    % Standard Steady State Radius R (Eq 2)
    if abs(delta_prev) < 1e-5
        R = inf;
    else
        R = (L + Kug * u^2) / delta_prev;
    end
    
    % Heading/Normal vectors
    tG = [cos(psi), sin(psi)];
    nG = [-sin(psi), cos(psi)]; % Normal to Left
    
    if isinf(R)
        % Straight line projection
        P = [X, Y] + u * Tp * tG;
    else
        % Circular projection (Eq 5)
        % Center of curvature O = G + R * nG
        O = [X, Y] + R * nG;
        
        % Angle traversed theta (Eq 4)
        theta = (u * Tp) / R;
        
        % Vector from O to G is -R*nG. Rotate it by theta.
        v = -R * nG;
        
        cos_t = cos(theta);
        sin_t = sin(theta);
        
        % 2D Rotation
        v_rot = [v(1)*cos_t - v(2)*sin_t, v(1)*sin_t + v(2)*cos_t];
        
        P = O + v_rot;
    end
    
    % Lateral Error Calculation
    % Find segment P falls on.
    % Search forward from 'next_idx' since P is ahead.
    
    best_d = inf;
    best_idx = next_idx;
    found_valid = false;
    
    search_fwd = 200; % Larger window for lookahead
    
    for k = next_idx : min(next_idx + search_fwd, N-1)
        curr_S = road(k, :);
        next_S = road(k+1, :);
        
        seg_vec = next_S - curr_S;
        s_len = norm(seg_vec);
        if s_len < 1e-6, continue; end
        t_hat_L = seg_vec / s_len;
        
        p_vec = P - curr_S;
        proj = dot(p_vec, t_hat_L);
        
        % Check if valid segment (Eq 9)
        if proj >= -1.0 && proj <= s_len + 1.0 % Allow 1m slack
             % Valid segment
             n_L_local = [-t_hat_L(2), t_hat_L(1)]; % Left Normal
             dist = dot(p_vec, n_L_local);
             
             % Find smallest valid d (closest segment involved)
             if abs(dist) < abs(best_d)
                 best_d = dist;
                 best_idx = k;
                 found_valid = true;
             end
        end
    end
    
    if ~found_valid
        % Fallback: P is likely far outside track or off the end
        % Use closest point logic from next_idx
        p_start = road(next_idx, :);
        dist_direct = norm(P - p_start);
                
        % Projection onto the 'next_idx' tangent extended.
        curr_S = road(next_idx, :);
        next_S = road(next_idx+1, :);
        seg_vec = next_S - curr_S;
        t_hat_L = seg_vec / norm(seg_vec);
        n_L_local = [-t_hat_L(2), t_hat_L(1)];
        best_d = dot(P - curr_S, n_L_local);
    end
    
    d_lat = best_d;
    
    debug_info.P = P;
    debug_info.d_lat = d_lat;
    
    %Steering Update (Eq 11) 
    % delta(k+1) = delta(k) - Klat * d
    delta_cmd = delta_prev - Klat * d_lat;
    
    % Saturation
    delta_max = deg2rad(35);
    delta_cmd = max(-delta_max, min(delta_cmd, delta_max));