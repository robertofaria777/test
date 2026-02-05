function [delta_cmd, next_idx] = driver_model_paper(X, Y, psi, u, delta_prev, road, params, prev_idx)
    % DRIVER_MODEL_PAPER - Steering control based on provided paper images
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
    
    % --- Extract Parameters ---
    Tp = params.Tp;       % Preview time (s)
    Klat = params.Klat;   % Lateral gain
    
    % --- 1. Find Current Position S_G on Track ---
    % Instead of searching all points, start from prev_idx and look forward
    % Search a small window ahead to account for large time steps or fast cars
    search_window = 50; 
    N = size(road, 1);
    
    idx = prev_idx;
    min_dist = inf;
    
    % Basic search for closest point (simplification of 'valid segment' logic for robust tracking)
    for i = prev_idx : min(prev_idx + search_window, N-1)
        p1 = road(i, :);
        p2 = road(i+1, :);
        
        % Vector from p1 to p2 (Tangent)
        t_vec = p2 - p1;
        seg_len = norm(t_vec);
        if seg_len < 1e-6, continue; end
        t_hat = t_vec / seg_len;
        
        % Vector from p1 to Vehicle
        v_vec = [X, Y] - p1;
        
        % Project vehicle onto line segment to find distance along segment
        proj_len = dot(v_vec, t_hat);
        
        % Check if projection falls within segment [0, seg_len]
        % We check strict bounds, but allow slight tolerance
        if proj_len >= 0 && proj_len <= seg_len
             d = norm(v_vec - proj_len * t_hat);
             if d < min_dist
                 min_dist = d;
                 idx = i;
             end
        end
    end
    
    % Current segment start S_L
    S_L = road(idx, :);
    next_idx = idx; % Update state for next iteration
    
    % --- 2. Calculate Preview Point P ---
    % Distance to look ahead
    previewDist = max(1.0, u * Tp);
    
    travelled = 0;
    j = idx;
    
    % First, finish the remaining part of current segment
    p1 = road(j, :);
    p2 = road(j+1, :);
    t_vec = p2 - p1;
    seg_len = norm(t_vec);
    t_hat = t_vec / seg_len;
    
    % Distance vehicle is along the current segment
    dist_on_seg = dot([X, Y] - p1, t_hat);
    dist_remaining = seg_len - dist_on_seg;
    
    if previewDist <= dist_remaining
        % Preview point is on the current segment
        P = [X, Y] + (previewDist * p1_to_car_dir) % Approximation
        % BETTER: P is just further down the centerline from projection
        P = p1 + (dist_on_seg + previewDist) * t_hat;
    else
        travelled = dist_remaining;
        j = j + 1;
        
        while j < N && travelled < previewDist
            p_start = road(j, :);
            p_end   = road(j+1, :);
            seg_len = norm(p_end - p_start);
            
            if travelled + seg_len >= previewDist
                % Point is on this segment
                remaining = previewDist - travelled;
                t_hat = (p_end - p_start) / seg_len;
                P = p_start + remaining * t_hat;
                travelled = previewDist + 1; % break
            else
                travelled = travelled + seg_len;
                j = j + 1;
            end
        end
    end
    
    if j >= N
       P = road(end, :); 
    end

    % --- 3. Lateral Error Calculation ---
    % Find segment P falls on (re-using j logic effectively implies we know P's segment)
    % The paper method (Eq 8): d_L = (P - S_L).n_L
    % Where S_L is the start of the valid segment for P.
    % Note: The Paper Fig 1 implies error is calculated relative to the LINEAR SEGMENT track.
    
    % Since we found P by walking segments, 'j' (or 'j-1' if loop broke early) is the segment.
    % If loop broke at 'travelled = previewDist + 1', P is on segment 'j'.
    % If loop ran out, P is endpoint.
    if j < N
        seg_idx = j;
    else
        seg_idx = N-1;
    end
    
    S_L_preview = road(seg_idx, :);
    next_pt     = road(seg_idx+1, :);
    
    seg_vec = next_pt - S_L_preview;
    seg_len = norm(seg_vec);
    t_hat_L = seg_vec / seg_len;
    
    % Normal vector n_L (Rotate tangent -90 degrees? Or 90?)
    % Paper Fig 1: t is forward, n is Left.
    % If t = [tx, ty], n_L = [-ty, tx] (Left turn: x=1,y=0 -> t=[1,0], n=[0,1])
    n_L = [-t_hat_L(2), t_hat_L(1)];
    
    % Error vector from Segment Start to Preview Point P ( Wait, P is ON the line by definition of step 2)
    % The paper says: "d_L = (P - S_L).n_L"
    % But P is calculated *along the track* in most preview models.
    % OR, is P defined as the "point at distance Tp * u projected from Vehicle heading?"
    % Let's re-read the paper snippet text.
    % "Lateral control is based on steering corrections aimed at projecting the vehicle onto a path at a single preview point on the road ahead."
    % "projecting the vehicle ... to a path" implies P is on the ROAD.
    % BUT Eq 5 defines P via: P = G + R n_G ...
    % Eq 6: dc = Rc - |P - Oc|
    % This implies P is calculated from VEHICLE kinematics (Projected position)?
    % Text: "Predict forward path radius... The control operates discretely... The forward path radius... is R = ... (Eq 2)"
    % Then it talks about "One way to locate point P is via the arc centre O... P = G + R n_G ..." 
    % SO:
    % 1. P is the PREDICTED vehicle position at time Tp.
    % 2. d_L is the distance from this PREDICTED P to the track.
    
    % Let's implement that interpretation.
    
    % Step 3a: Calculate Steady State Radius R (Eq 2)
    % R = (L + Kug * u^2) / delta
    % Avoid divide by zero
    if abs(delta_prev) < 1e-4
        R = inf;
    else
        % We need L and Kug from params. let's assume standard values if not present
        L = 2.5; % Wheelbase
        Kug = 0; % Understeer gradient
        if isfield(params, 'L'), L = params.L; end
        if isfield(params, 'Kug'), Kug = params.Kug; end
        
        R = (L + Kug * u^2) / delta_prev;
    end
    
    % Step 3b: Locate P (Predicted Vehicle Pos) using constant turn
    % Heading vector tG
    tG = [cos(psi), sin(psi)];
    % Normal vector nG (Left)
    nG = [-sin(psi), cos(psi)];
    
    if isinf(R)
        % Straight line projection
        P = [X, Y] + u * Tp * tG;
    else
        % Circular projection (Eq 5)
        % Center of curvature O?
        % Vector from G to O is R * nG (if R > 0 turns left?)
        % Standard: delta > 0 is Left Turn -> R > 0. Center is to the Left.
        O = [X, Y] + R * nG;
        
        % Angle traversed theta (Eq 4)
        theta = (u * Tp) / R;
        
        % Rotation matrix for vector (G-O) -> (P-O)
        % The vector from O to G is -R*nG
        % We rotate this by theta? 
        % Paper Eq 5: P = G + R n_G - [Rot] R n_G
        % Essentially P is rotation of G around O.
        
        % Let's use simple vector rotation
        % v = G - O = -R * nG
        v = -R * nG;
        
        % Rotate v by theta
        cos_t = cos(theta);
        sin_t = sin(theta);
        
        % 2D Rotation: x' = x cos - y sin, y' = x sin + y cos
        v_rot = [v(1)*cos_t - v(2)*sin_t, v(1)*sin_t + v(2)*cos_t];
        
        P = O + v_rot;
    end
    
    % --- 4. Find Track Segment closest to Predicted P ---
    % search_window matches previous logic, starting from likely current segment
    % We reuse 'next_idx' found earlier for vehicle position as a start point
    best_d = inf;
    best_idx = next_idx;
    best_n_L = [0, 1];
    best_S_L = [0, 0];
    
    % Look ahead from car position
    for k = next_idx : min(next_idx + 100, N-1)
        curr_S = road(k, :);
        next_S = road(k+1, :);
        
        seg_vec = next_S - curr_S;
        s_len = norm(seg_vec);
        if s_len < 1e-6, continue; end
        t_hat_L = seg_vec / s_len;
        
        % Check if P projects onto this segment (Eq 9: 0 < (P-SL).tl < |rL|)
        p_vec = P - curr_S;
        proj = dot(p_vec, t_hat_L);
        
        if proj >= 0 && proj <= s_len
             % Valid segment
             % Calculate signed distance (Eq 8)
             n_L_local = [-t_hat_L(2), t_hat_L(1)];
             dist = dot(p_vec, n_L_local);
             
             % "Smallest valid dL will be appropriate"
             if abs(dist) < abs(best_d)
                 best_d = dist;
                 best_idx = k;
                 best_n_L = n_L_local;
                 best_S_L = curr_S;
             end
        end
    end
    
    if isinf(best_d)
        % Fallback: just separate distance to end or closest point
        best_d = 0; % Fail safe
    end
    
    d_lat = best_d;
    
    debug_info.P = P;
    debug_info.d_lat = d_lat;
    
    % --- 5. Steering Update (Eq 11) ---
    % delta(k+1) = delta(k) - Klat * d
    delta_cmd = delta_prev - Klat * d_lat;
    
    % --- 6. Saturation ---
    delta_max = deg2rad(35);
    delta_cmd = max(-delta_max, min(delta_cmd, delta_max));
    
end
