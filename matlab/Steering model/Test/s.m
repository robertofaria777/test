clear; clc; close all;

% --- road
roadY = linspace(0,1000,2001)';       
roadX = zeros(size(roadY));
road  = [roadX roadY];

% --- vehicle initial state
startpos = [2, 0];
X   = startpos(1);
Y   = startpos(2);
psi = pi/2;                               % facing +Y (upwards) (90 degree)
u   = 10;                                 % speed m/s
delta = 0;

% --- driver params ---
params.Tp   = 0.5;                        % preview time
params.Klat = 0.2;                        % lateral gain
params.L    = 2.6;                        % wheelbase

% --- simulation settings ---
dt = 0.01;      %Time interval
T  = 100;       %Time seconds
N  = round(T/dt);
path = zeros(N,2);

for k = 1:N
    delta = steering_model(X, Y, psi, u, delta, road, params);

    % simple kinematics without understeer (slip)
    X   = X + u*cos(psi)*dt;
    Y   = Y + u*sin(psi)*dt;
    psi = psi + (u/params.L)*tan(delta)*dt;

    path(k,:) = [X Y];
end

% --- plot ---
plot(road(:,1), road(:,2), 'k--','LineWidth',2); hold on;
plot(path(:,1), path(:,2), 'r','LineWidth',2);
plot(startpos(1),startpos(2),'rx','MarkerSize',10,'LineWidth',2);
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
legend('Road x=0','Vehicle path','Location','NorthWest');
title('AVEC-style preview steering to follow x=0');
