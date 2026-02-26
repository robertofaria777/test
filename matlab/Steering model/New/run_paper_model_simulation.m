% RUN_PAPER_MODEL_SIMULATION

clear; clc; close all;

%Track Generation
filename = 'C:\Users\User\Desktop\APP Development\test\matlab\Steering model\New\monza-graphic.jpeg';

    rgb = imread(filename);
    
    Red   = rgb(:,:,1);
    Green = rgb(:,:,2);
    Blue  = rgb(:,:,3);
    
    % Binary mask
    bw = Blue > 150 & Red < 128;
    
    % Extract boundaries
    B = bwboundaries(bw);
    
    % Use only one boundary (e.g. the first)
    boundary = B{1};
    
    % Original coordinates
    x = boundary(:,2);
    y = boundary(:,1);
    
    % 1) Moving average smoothing
    windowSize = 25;   % Smooth out the jagged pixel edges
    x_smooth = movmean(x, windowSize);
    y_smooth = movmean(y, windowSize);
    
    % 2) Increase resolution by interpolation
    N_points = numel(x_smooth);
    t = 1:N_points;                
    ti = linspace(1, N_points, N_points * 10); % Upsample 10x
    
    % Smooth interpolation
    roadX = interp1(t, x_smooth, ti, 'pchip')';
    roadY = interp1(t, y_smooth, ti, 'pchip')';
    
    

road = [roadX roadY];

%Simulation Setup
% Set initial state near the start of the loaded track
start_idx = 1;
startpos = road(start_idx, :);

X   = startpos(1);
Y   = startpos(2);

% Calculate initial heading based on track tangent
p1 = road(start_idx, :);
p2 = road(min(start_idx+5, size(road,1)), :); % Look a few points ahead for tangent
psi = atan2(p2(2)-p1(2), p2(1)-p1(1)); 

u   = 20;    % speed m/s
delta = 0; % steering angle

% Parameters
params.Tp   = 0.5;    % Used in the paper
params.Klat = 0.1;    % Lower gain for stability
params.L    = 2.6;    % Wheelbase 
params.Kug  = 0.0; 

% Simulation Settings
dt = 0.01;        % Time interval 
T  = 500;         % Duration
N_sim  = round(T/dt);

% Storage
history.x = zeros(N_sim, 1);
history.y = zeros(N_sim, 1);
history.psi = zeros(N_sim, 1);
history.delta = zeros(N_sim, 1);

prev_idx = 1;

% Setup Figure
figure('Name', 'Steering Control', 'Color', 'w');
plot(road(:,1), road(:,2), 'k--','LineWidth',2); hold on;
hCar = plot(X, Y, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6);
hPred = plot(X, Y, 'gx', 'MarkerSize', 8, 'LineWidth', 2);
axis equal; grid on; set(gca,'YDir','reverse'); % Image coordinates often need reverse Y
xlabel('X (m)'); ylabel('Y (m)');
legend('Road','Vehicle','Preview P','Location','best');
title('Steering Control - Paper Model');

disp('Starting simulation...');

for k = 1:N_sim
    history.x(k) = X;
    history.y(k) = Y;
    history.psi(k) = psi;
    history.delta(k) = delta;
    
    % --- Call Driver Model ---
    [delta_cmd, prev_idx, debug] = driver_model_paper(X, Y, psi, u, delta, road, params, prev_idx);
    
    % --- Vehicle Dynamics ---
    % Standard kinematic bicycle
    % x_dot = u * cos(psi)
    % y_dot = u * sin(psi)
    % psi_dot = (u/L) * tan(delta)
    
    X   = X + u*cos(psi)*dt;
    Y   = Y + u*sin(psi)*dt;
    psi = psi + (u/params.L)*tan(delta)*dt;
    
    delta = delta_cmd; % No actuator lag in user snippet
    
    % Visualization
    if mod(k, 10) == 0
        set(hCar, 'XData', X, 'YData', Y);
        if isfield(debug, 'P')
            set(hPred, 'XData', debug.P(1), 'YData', debug.P(2));
        end
        drawnow limitrate;
    end
    
    % Stop if completed lap (approx)
    if k > 500 && norm([X,Y] - startpos) < 5
        disp('Lap completed.');
        break;
    end
end

% Plot Trajectory
plot(history.x(1:k), history.y(1:k), 'r-', 'LineWidth', 1.5);
disp('Done.');
