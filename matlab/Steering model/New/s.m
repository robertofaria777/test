clear; clc; close all;
%Race track
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

%% 1) Moving average smoothing
windowSize = 5;   % adjust as needed (must be >= 1)
x_smooth = movmean(x, windowSize);
y_smooth = movmean(y, windowSize);

%% 2) Increase resolution by interpolation
N = numel(x_smooth);
t = 1:N;                % parameter along the curve
upFactor = 10;          % how many times more points you want
ti = linspace(1, N, N*upFactor);

% Smooth interpolation (piecewise cubic)
x_highres = interp1(t, x_smooth, ti, 'pchip');
y_highres = interp1(t, y_smooth, ti, 'pchip');

%% Optional: display

%figure;
%imshow(rgb); hold on;

% Original boundary (faint)
%plot(x, y, '.', 'MarkerSize', 4);

% High-res smoothed boundary (line)
%plot(x_highres, y_highres, 'LineWidth', 2);

%set(gca, 'YDir', 'reverse');
%title('Smoothed & High-Resolution Boundary');

% If you only want the coordinates (no image), comment out imshow and plotting
% and just use x_highres, y_highres as your new line:
roadX = x_smooth;
roadY = y_smooth;

% road
%roadY = linspace(0,1000,2001)';       
%roadX = zeros(size(roadY));
road  = [roadX roadY];

% vehicle initial state
startpos = [99, 117];
X   = startpos(1);
Y   = startpos(2);
psi = pi/2;                               % facing +Y (upwards) (90 degree)
u   = 10;                                 % speed m/s
delta = 0;

% driver params (need to change later)
params.Tp   = 0.5;                        % preview time
params.Klat = 0.2;                        % lateral gain
params.L    = 2.6;                        % wheelbase

% simulation settings
dt = 0.01;      %Time interval
T  = 500;       %Time seconds
N  = round(T/dt);
path = zeros(N,2);

for k = 1:N
    delta = steering_model(X, Y, psi, u, delta, road, params);

    % simple kinematics without understeer and slip
    X   = X + u*cos(psi)*dt;
    Y   = Y + u*sin(psi)*dt;
    psi = psi + (u/params.L)*tan(delta)*dt;

    path(k,:) = [X Y];
end

% plot
plot(road(:,1), road(:,2), 'k--','LineWidth',2); hold on;
plot(path(:,1), path(:,2), 'r','LineWidth',2);
plot(startpos(1),startpos(2),'rx','MarkerSize',10,'LineWidth',2);
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
legend('Road','Vehicle path','Location','NorthWest');
title('Steering control');

