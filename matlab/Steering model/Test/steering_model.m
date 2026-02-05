function delta_cmd = steering_model(X, Y, psi, u, delta_prev, road, params)

    g = 9.81;

    % Driver parameters (need to change later)
    if isfield(params,'Tp'),   Tp   = params.Tp;   else, Tp   = 0.5;   end   % preview time [s]
    if isfield(params,'Klat'), Klat = params.Klat; else, Klat = 0.2;   end   % lateral gain

    % preview distance along the path
    previewDist = max(1.0, u * Tp);     % [m]

    % ------------ 1. Find closest point on road -------------------------
    d_all = vecnorm(road - [X Y], 2, 2);   % distance to each road point
    [~, idx] = min(d_all);
    idx = max(1, min(idx, size(road,1)-1));  % keep a valid segment index

    % ------------ 2. March forward to preview point ---------------------
    travelled = 0;
    j = idx;
    while j < size(road,1) && travelled < previewDist
        ds = norm(road(j+1,:) - road(j,:));
        travelled = travelled + ds;
        j = j + 1;
    end
    P = road(j,:);   % preview point on road centreline

    % ------------ 3. Vehicle unit vectors -------------------------------
    % Forward direction (tangent of vehicle)
    tG = [cos(psi), sin(psi)];
    % Left normal of vehicle
    nG = [-sin(psi), cos(psi)];

    % ------------ 4. Lateral error at preview point ---------------------
    e = P - [X Y];          % vector from vehicle to preview point
    % Dot with left-normal gives signed lateral error in "vehicle left" dir
    % For AVEC law we want: road on LEFT → d < 0 so δ increases (left steer)
    d_lat = -dot(e, nG);    % note the minus sign (important!)

    % ------------ 5. AVEC steering correction ---------------------------
    delta_cmd = delta_prev - Klat * d_lat;

    % ------------ 6. Saturation -----------------------------------------
    delta_max = deg2rad(35);
    delta_cmd = min(max(delta_cmd, -delta_max), delta_max);
end
