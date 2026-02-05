function delta_cmd = steering_model(X, Y, psi, u, delta_prev, road, params) 

    g = 9.81; %gravity

    % Driver parameters (need to change later)
    Tp   = 2.5;   % preview time seconds
    Klat = 0.2;   % lateral gain

    % preview distance along the path
    previewDist = max(1.0, u * Tp);     %it changes acording to the speed u times the preview time Tp

    %1) Find closest point on road
    d_all = vecnorm(road - [X Y], 2, 2);   % distance to each road point
    [~, idx] = min(d_all);
    idx = max(1, min(idx, size(road,1)-1));  % keep a valid segment index

    %2) Move forward to preview point
    travelled = 0;
    j = idx;
    while j < size(road,1) && travelled < previewDist
        ds = norm(road(j+1,:) - road(j,:));
        travelled = travelled + ds;
        j = j + 1;
    end
    P = road(j,:);   % preview point on road centreline

    %3) Vehicle unit vectors
    % Forward direction (tangent of vehicle)
    tG = [cos(psi), sin(psi)];
    % Left normal of vehicle
    nG = [-sin(psi), cos(psi)];

    %4) Lateral error at preview point
    e = P - [X Y];          % vector from vehicle to preview point
    %Road on LEFT, d < 0 so δ increases (left steer)
    d_lat = -dot(e, nG);    

    %5) Steering correction
    delta_cmd = delta_prev - Klat * d_lat;

    %6) Saturation
    delta_max = deg2rad(35);
    delta_cmd = min(max(delta_cmd, -delta_max), delta_max);
end
