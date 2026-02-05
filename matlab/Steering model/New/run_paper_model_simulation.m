% RUN_PAPER_MODEL_SIMULATION
% This script runs the simulation for the 'driver_model_paper.m'
% It attempts to load the track from 'racetrack2.m', or falls back to a default oval.

clear; clc; close all;

% --- 1. Load Track ---
try
    % Try extracting track from the user's file
    disp('Attempting to run racetrack2.m to load track...');
    racetrack2; 
    
    % racetrack2 produces 'x' and 'y' vectors
    road_x = x;
    road_y = y;
    disp('Track loaded efficiently from racetrack2.');
catch ME
    warning('Could not run racetrack2.m (possibly missing image file). Using default oval track.');
    % Fallback: Generate an oval track
    t = linspace(0, 2*pi, 200)';
    road_x = 100 * cos(t);
    road_y = 50 * sin(t);
end

% Combine into road matrix [x, y]
road = [road_x, road_y];

% Smooth/Interpolate if necessary (optional)
% Ensure road is dense enough for the model logic
if size(road,1) < 100
   % Simple interpolation
   t_old = 1:size(road,1);
   t_new = linspace(1, size(road,1), 500)';
   road = interp1(t_old, road, t_new, 'spline');
end

% --- 2. Simulation Setup ---
% Initial Vehicle State (Start at beginning of road)
X = road(1, 1);
Y = road(1, 2);
psi = atan2(road(2,2)-road(1,2), road(2,1)-road(1,1)); % Aligned with track
u = 20; % Constant speed 20 m/s
delta = 0; % Steering angle

% Parameters
params.Tp = 0.8;    % Preview time (s)
params.Klat = 0.15; % Lateral gain
params.L = 2.5;     % Wheelbase (m)
params.Kug = 0.0;   % Understeer gradient

% Simulation Loop
dt = 0.05;
T_final = 60; % seconds
N_steps = round(T_final / dt);

% Storage for Plotting
history.x = zeros(N_steps, 1);
history.y = zeros(N_steps, 1);
history.psi = zeros(N_steps, 1);
history.delta = zeros(N_steps, 1);
history.preview_p = zeros(N_steps, 2);

prev_idx = 1; % For optimization in driver model

figure('Name', 'Steering Model Simulation', 'Color', 'w');
plot(road(:,1), road(:,2), 'k--', 'LineWidth', 2); hold on;
hCar = plot(X, Y, 'bo', 'MarkerFaceColor', 'b');
hPred = plot(X, Y, 'gx', 'MarkerSize', 8);
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('Paper Driver Model Simulation');

disp('Starting simulation...');

for k = 1:N_steps
    % 1. Store History
    history.x(k) = X;
    history.y(k) = Y;
    history.psi(k) = psi;
    history.delta(k) = delta;
    
    % 2. Call Driver Model
    % [delta_cmd, next_idx] = driver_model_paper(X, Y, psi, u, delta_prev, road, params, prev_idx)
    [delta_cmd, prev_idx] = driver_model_paper(X, Y, psi, u, delta, road, params, prev_idx);
    
    % 3. Vehicle Dynamics (Kinematic Bicycle)
    % x_dot = u * cos(psi)
    % y_dot = u * sin(psi)
    % psi_dot = (u/L) * tan(delta)
    
    X = X + u * cos(psi) * dt;
    Y = Y + u * sin(psi) * dt;
    psi = psi + (u / params.L) * tan(delta) * dt;
    
    % Actuator dynamics (simple lag) or direct assignment
    delta = delta_cmd; 
    
    % 4. Visualization Update (every 10 steps for speed)
    if mod(k, 5) == 0
        set(hCar, 'XData', X, 'YData', Y);
        
        % Re-calculate P just for plotting standard visualization (driver_model calculates it internally)
        % For speed, we won't extract P from the function unless we modify the function to return it.
        % Let's just draw the car.
        drawnow limitrate;
    end
    
    % Stop if we looped roughly (simple check)
    if k > 200 && norm([X,Y] - road(1,:)) < 5
        disp('Lap Completed!');
        break;
    end
end

% Final Plot
plot(history.x(1:k), history.y(1:k), 'b-', 'LineWidth', 1.5);
legend('Track', 'Final Position', 'Start', 'Trajectory');
disp('Simulation Finished.');
