figure; hold on; axis equal; grid on;

% Road corridor (5m wide = 2.5m each side)
hw = 2.5;  % half-width in metres
dx = diff(road(:,1));  dy = diff(road(:,2));
len = sqrt(dx.^2 + dy.^2);  len(len<1e-9) = 1e-9;
% Left/right normals
nx = [-dy; -dy(end)] ./ [len; len(end)];
ny = [ dx;  dx(end)] ./ [len; len(end)];
Lx = road(:,1) + hw*nx;  Ly = road(:,2) + hw*ny;
Rx = road(:,1) - hw*nx;  Ry = road(:,2) - hw*ny;
fill([Lx; flipud(Rx)], [Ly; flipud(Ry)], ...
     [0.75 0.75 0.75], 'EdgeColor','none', 'DisplayName','Road (5m)');
plot(road(:,1), road(:,2), 'k--', 'LineWidth',1, 'DisplayName','Centreline');

% Vehicle path
% If X/Y are timeseries objects use X.Data, otherwise use directly
if isstruct(X), vx = X.Data(:); vy = Y.Data(:);
else,           vx = X(:);      vy = Y(:);  end
plot(vx, vy, 'b-', 'LineWidth', 2, 'DisplayName','Vehicle path');
plot(vx(1), vy(1), 'go', 'MarkerSize',10, 'MarkerFaceColor','g', 'DisplayName','Start');
plot(vx(end), vy(end), 'rs', 'MarkerSize',10, 'MarkerFaceColor','r', 'DisplayName','End');

legend; title('Vehicle Path vs Track'); xlabel('X (m)'); ylabel('Y (m)');
