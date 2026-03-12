%% data_analysis.m
% Analyzes and plots simulation results from the 6-DOF handling model.
%
% Profiles (against distance):
%   Fig 1: Speed, Longitudinal Acceleration, Lateral Acceleration (3-in-1 subplot)
%   Fig 2: Steering Angle, Wheel Spin
%   Fig 3: Track Position Map (Color-coded by speed)
%
% This script handles both vector and Simulink timeseries (struct) data.

clearvars -except X Y u delta torq wvel road psi; % keep main results
close all; clc;

%% 1. Data Retrieval and Normalization
% Check for required variables in the workspace
required_vars = {'X', 'Y', 'u', 'delta'};
for i = 1:length(required_vars)
    if ~exist(required_vars{i}, 'var')
        error('Variable "%s" not found. Run a simulation first.', required_vars{i});
    end
end

x_pos = getData(X);
y_pos = getData(Y);
vel   = getData(u);
steer_all = getData(delta); % can be Nx4 for steering 4 wheels

% Take the first column (front-left wheel) if multiple columns exist
if size(steer_all, 2) > 1, steer = steer_all(:,1); else, steer = steer_all; end

if exist('psi', 'var')
    yaw = getData(psi);
else
    % Calculate heading from X, Y
    dx_raw = [0; diff(x_pos)];
    dy_raw = [0; diff(y_pos)];
    yaw = atan2(dy_raw, dx_raw);
end

% Optional: Wheel Velocities for Spin calculation
if exist('wvel', 'var')
    w_vel = getData(wvel);
else
    w_vel = zeros(length(x_pos), 4);
    warning('Variable "wvel" not found. Using zero.');
end

% Ensure columns for wheel velocity
if size(w_vel, 2) < 4 && size(w_vel, 1) == 4, w_vel = w_vel'; end

%% 2. Derived Calculations
% --- Distance Calculation ---
dx = [0; diff(x_pos)];
dy = [0; diff(y_pos)];
ds = sqrt(dx.^2 + dy.^2);
dist = cumsum(ds); % cumulative distance along the path

% --- Acceleration Calculation ---
% Assuming dt = 0.01 (matches init6dof_v4)
dt_sim = 0.01; 
if length(dist) > 1
    % Raw acceleration from differentiation
    ax_raw = [0; diff(vel)] / dt_sim;

    % Smooth yaw before differentiating to avoid atan2 jumps
    dpsi = diff(unwrap(yaw));
    ay_raw = [0; vel(2:end) .* (dpsi / dt_sim)];

    % Moving average to remove noise (window = 200 samples = 2s)
    win = 200;
    ax = movmean(ax_raw, win);
    ay = movmean(ay_raw, win);
else
    ax = zeros(size(x_pos));
    ay = zeros(size(x_pos));
end

% --- Wheel Spin ---
% Calculation: Difference between front-left and rear-left wheel speeds (rad/s)
rr = 0.3; % rolling radius
if size(w_vel, 2) >= 3
    v_wheels = w_vel * rr; % convert rad/s to m/s
    wheel_spin = abs(v_wheels(:,1) - v_wheels(:,3)); % FL - RL difference
else
    wheel_spin = zeros(size(x_pos));
end

%% 3. Plotting
% --- FIGURE 1: Dynamic Performance (3 variables) ---
fig1 = figure('Name', 'Fig 1: Speed & Acceleration vs Distance', 'Units', 'normalized', 'Position', [0.1 0.5 0.4 0.4]);
subplot(3, 1, 1);
plot(dist, vel * 3.6, 'b-', 'LineWidth', 2);
grid on; ylabel('Speed [km/h]'); title('Speed Profile');

subplot(3, 1, 2);
plot(dist, ax, 'g-', 'LineWidth', 1.5);
grid on; ylabel('Long Accel [m/s^2]'); title('Longitudinal Acceleration (Throttle/Brake)');

subplot(3, 1, 3);
plot(dist, ay, 'r-', 'LineWidth', 1.5);
grid on; ylabel('Lat Accel [m/s^2]'); title('Lateral Acceleration (Cornering)');
xlabel('Distance [m]');

linkaxes(findobj(fig1, 'Type', 'axes'), 'x');
xlim([0, max(dist)]);

% --- FIGURE 2: Control & Handling ---
fig2 = figure('Name', 'Fig 2: Steering & Wheel Spin vs Distance', 'Units', 'normalized', 'Position', [0.5 0.5 0.4 0.4]);
subplot(2, 1, 1);
plot(dist, rad2deg(steer), 'Color', [0.85 0.325 0.098], 'LineWidth', 1.5);
grid on; ylabel('Angle [deg]'); title('Steering Input (Front-Left)');

subplot(2, 1, 2);
plot(dist, wheel_spin * 3.6, 'm-', 'LineWidth', 1.5);
grid on; ylabel('Slip [km/h]'); title('Wheel Spin (FL vs RL)');
xlabel('Distance [m]');

linkaxes(findobj(fig2, 'Type', 'axes'), 'x');
xlim([0, max(dist)]);

% --- FIGURE 3: Track Position Map ---
fig3 = figure('Name', 'Fig 3: Track Position Map', 'Units', 'normalized', 'Position', [0.3 0.1 0.4 0.4]);
if exist('road', 'var') && ~isempty(road)
    plot(road(:,1), road(:,2), 'k--', 'LineWidth', 0.5, 'DisplayName', 'Track Layout'); hold on;
end
% Color-code the path by speed to see where the car is slow/fast
scatter(x_pos, y_pos, 10, vel * 3.6, 'filled', 'DisplayName', 'Car Position');
colormap(jet); hc = colorbar; ylabel(hc, 'Speed [km/h]');
axis equal; grid on; title('Car Position on Track');
xlabel('X [m]'); ylabel('Y [m]');
legend('Location', 'best');

fprintf('Analysis Complete. Generated 3 figures plotted against distance.\n');

%% --- Local Functions ---
function out = getData(v)
    % Extracts data from a variable. 
    % 1. Handles Simulink Timeseries (struct or object with .Data)
    % 2. Handles numeric vectors and matrices (ensuring correct orientation)
    
    if isstruct(v) && isfield(v, 'Data')
        out = v.Data;
    elseif isstruct(v) && isprop(v, 'Data')
        out = v.Data;
    else
        out = v; % Already numeric
    end

    % If it's a column matrix (4xN), transpose it to Row-per-time (Nx4)
    if size(out, 1) == 4 && size(out, 2) > 4
        out = out';
    end
end
