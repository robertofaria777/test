%% test_cruisecontrol.m
% Standalone test for Cruise_control.m on a perfect 100 m radius circle.
%
% The function only outputs target speed (u_target) – the existing Simulink
% PI controller (Kc, Ic) handles the torque from there.
%
% Analytic expectation:
%   u_target = sqrt(ay_max * R) = sqrt(7.5 * 100) ≈ 27.39 m/s  ≈ 98.6 km/h

clear; clc; close all;

%% ---- Hardcoded parameters ------------------------------------------
R_true   = 100;      % [m]  circle radius
n_pts    = 2000;     % number of road points

cruise_params.Tp     = 3.0;   % [s]   lookahead time
cruise_params.ay_max = 7.5;   % [m/s²]
cruise_params.u_min  = 2.0;   % [m/s] minimum speed floor

%% ---- Build circular road -------------------------------------------
theta = linspace(0, 2*pi, n_pts)';
road  = R_true * [cos(theta), sin(theta)];

%% ---- Vehicle initial state -----------------------------------------
% Placed at (R, 0), heading north (psi = pi/2) – tangent to the circle.
X        =  R_true;
Y        =  0;
psi      =  pi/2;
u_curr   =  10;     % [m/s]  current speed (arbitrary – not used in speed calc)
prev_idx =  1;

%% ---- Call Cruise_control -------------------------------------------
[u_target, next_idx] = Cruise_control(psi, X, Y, u_curr, cruise_params, prev_idx, road);

%% ---- Analytic reference --------------------------------------------
u_expected = sqrt(cruise_params.ay_max * R_true);   % 27.39 m/s
err_pct    = abs(u_target - u_expected) / u_expected * 100;
pass       = err_pct < 5.0;   % 5 % tolerance (point-cloud discretisation)

%% ---- Print results -------------------------------------------------
fprintf('==============================================\n');
fprintf('  Cruise_control – 100 m circle test\n');
fprintf('==============================================\n');
fprintf('  Circle radius   : %g m\n',         R_true);
fprintf('  ay_max          : %.1f m/s²\n',    cruise_params.ay_max);
fprintf('  Expected u_target  : %.3f m/s  (%.1f km/h)\n', u_expected, u_expected*3.6);
fprintf('  Got      u_target  : %.3f m/s  (%.1f km/h)\n', u_target,      u_target*3.6);
fprintf('  Error           : %.2f %%\n',       err_pct);
fprintf('  Next road index : %d\n',            next_idx);
fprintf('----------------------------------------------\n');
if pass
    fprintf('  RESULT: PASS  ✓\n');
else
    fprintf('  RESULT: FAIL  ✗\n');
end
fprintf('==============================================\n');

%% ---- Plot ----------------------------------------------------------
figure('Name', 'Cruise Control – 100 m Circle Test', 'NumberTitle', 'off');

% Left panel: the track
subplot(1, 2, 1);
plot(road(:,1), road(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(X, Y, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
quiver(X, Y, 15*cos(psi), 15*sin(psi), 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
axis equal; grid on;
title('Track & Vehicle Start');
xlabel('X [m]'); ylabel('Y [m]');
legend('Road (100 m circle)', 'Vehicle pos', 'Heading', 'Location', 'best');

% Right panel: speed comparison bar
subplot(1, 2, 2); hold on;
vals = [u_target, u_expected] * 3.6;
b = bar(vals, 0.5);
b.FaceColor = 'flat';
b.CData = [0.2 0.7 0.4; 1 0.6 0.2];
set(gca, 'XTickLabel', {'u\_cmd (Cruise\_control)', 'Analytic ref'});
ylabel('Speed [km/h]');
title(sprintf('Target speed at R = %g m,  a_y = %.1f m/s²', R_true, cruise_params.ay_max));
grid on; ylim([0, max(vals)*1.3]);
for ii = 1:2
    text(ii, vals(ii) + 1, sprintf('%.1f km/h', vals(ii)), ...
         'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end
