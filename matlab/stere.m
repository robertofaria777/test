clear; clc; close all;

%% --------------------------------------------------------
% 1. Define Road (Simple Straight Line)
% ---------------------------------------------------------
road = [0 0;   0 1000];
plot(road(:,1), road(:,2), 'k--', 'LineWidth',2); hold on;
xlabel('X (m)'); ylabel('Y (m)'); grid on; axis equal;
title('Simple Steering Model Path Following');

%% --------------------------------------------------------
% 2. Start Position Off to the Side
% ---------------------------------------------------------
startpos = [2, 0];   % 2 m to the right of the road centerline
plot(startpos(1), startpos(2), 'rx', 'MarkerSize',10, 'LineWidth',2);

%% --------------------------------------------------------
% 3. Driver parameters (from AVEC model)
% ---------------------------------------------------------
params.Tp   = 0.5;      % preview time
params.Klat = 0.12;     % lateral steering gain
params.Kug  = 1;        % understeer gradient (deg/g)
params.L    = 2.6;      % wheelbase (m)

%% --------------------------------------------------------
% 4. Simulation settings
% ---------------------------------------------------------
dt = 0.01;      % timestep
T  = 10;        % total time
N  = round(T/dt);

% Vehicle initial state
X   = startpos(1);
Y   = startpos(2);
psi = 0;        % heading straight
u   = 10;       % 10 m/s constant speed
delta = 0;      % initial steering

% Store path
path = zeros(N,2);

%% --------------------------------------------------------
% 5. Simulation Loop
% ---------------------------------------------------------
for k = 1:N
    t = k*dt;   % current simulation time

    % Compute steering command
    delta = steering_model(X, Y, psi, u, delta, road, params);

    % Simple bicycle-model kinematics for motion
    beta = 0; % assume small slip for demo

    X = X + u*cos(psi)*dt;
    Y = Y + u*sin(psi)*dt;
    psi = psi + (u/params.L)*tan(delta)*dt;

    path(k,:) = [X Y];
    
    if abs(mod(t,1)) < dt
        plot(X, Y, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    end
end

%% --------------------------------------------------------
% 6. Plot the resulting driven path
% ---------------------------------------------------------
plot(path(:,1), path(:,2), 'b', 'LineWidth',2);
legend('Road Centerline','Start Position','Vehicle Path');
