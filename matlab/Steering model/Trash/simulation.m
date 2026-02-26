clear; clc; close all;

roadY = linspace(0,1000,2001)';
road = [zeros(size(roadY)) roadY];
%road = [0,0;0,1000];

plot(road(:,1),road(:,2),'k--','LineWidth',2); hold on;

startpos = [2,0];
plot(startpos(1),startpos(2),'rx','MarkerSize',10,'LineWidth',2);

params.L = 2.6;

dt = 0.01;
T  = 100;
N  = round(T/dt);
X=2;
Y=0;
%X = startpos(1);
%Y = startpos(2);
psi = pi/2;
u = 10;
delta = 0;

path = zeros(N,2);

for k = 1:N
    t = k*dt;

    delta = steering_model(X, Y, psi, u, delta, road, params);

    X = X + u*cos(psi)*dt;
    Y = Y + u*sin(psi)*dt;
    psi = psi + (u/params.L)*tan(delta)*dt;

    path(k,:) = [X Y];

    if abs(mod(t,1)) < dt
        plot(X,Y,'rx','MarkerSize',10,'LineWidth',2);
    end
end

plot(path(:,1),path(:,2),'b','LineWidth',2);
legend('Road','Path');
