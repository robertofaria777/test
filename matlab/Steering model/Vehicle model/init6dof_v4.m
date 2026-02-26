%
% Parameter file to support 6dof handling model.  
%
% NOMINAL SMALL FAMILY CAR


% INITIAL CONDITIONS (see also state and wheel speed i/c setting, below) :
u0 = 18;        
cruise = 1;    

% ADMINISTRATIVE INITIALISATION
o4 = ones(4,1);
TINY = 1e-5;

% ENVIRONMENTAL VARIABLES
g = 9.812;
mu = 0.99*o4;        % Base road surface friction
%mu(1) = 0.001;

% BODY MASS AND INERTIA
mf = 800;   % total sprung mass at front axle (kg)
mr = 600;   % total sprung mass at rear axle (kg)
ixx = 350;  % roll inertia about mass centre (kg m^2)
iyy = 2000; % pitch inertia about mass centre (kg m^2)
izz = 2200; % yaw inertia about mass centre (kg m^2)
ixz = 0;    % product of inertia

% GEOMETRY
l = 2.83;      % wheelbase (m)
hcg = 0.4;      % height of mass centre above ground (m)
hfrc = 0.1;     % height of front roll centre above ground (m)
hrrc = 0.4;     % height of rear roll centre above ground (m)
antidive = 0.0; % proportion of antidive in susp geometry (0-1)
antisquat = 0.0;% proportion of antisquat in susp geometry (0-1)

% SUSPENSION BASICS (wheel rates are assumed)
Bs = [1100 1100 1200 1200]; % front suspension damping rate (Ns/m)
Ks = [20 20 22 22]*1e3;     % suspension stiffness, equivalent, at corner (N/m)
stabf = 0;                % front anti roll bar (Nm/deg)  (400)
stabr = 0;                 % rear anti roll bar (Nm/deg)
ftrack = 1.5;             % front track
rtrack = 1.5;             % rear track


% TYRES (all tyres assumed the same, but could easily be set independently)
Kts =   [1.8 1.8 1.8 1.8]*1e5;  % vertical tyre stiffness   
c1s =   [3  3  3  3];     % c1 and c2 are cornering stiffness load variation coefficients
c2s =   [7  7  7  7];     % (see documentation, equation 9)
Zrds =  [16000  16000  16000  16000];   % 1400*10, reference (rated) weight (whole vehicle)
Cfacs = [1.5  1.5  1.5  1.5];           % multiplication factor for long. tyre stiffnesses
Es =    [-0.2  -0.2  -0.2  -0.2];       % Pacejka shape coefficient
Cs =    [1.4  1.4  1.4  1.4];           % Pacejka shape coefficient
tau =   0.025;                % slip delay for tyre forces

% DRIVELINE
rr = 0.3;                   % wheel rolling radius
%fdb = [0.5, 0.5, 0, 0];		% drive bias, proportion to each wheel (FWD)
fdb = [0, 0, 0.5, 0.5];		% drive bias, proportion to each wheel (RWD)
Iw  = [15 15 2 2];        % wheel (and nominal engine) moments of inertia (kg m^2)

% Cruise control proportional and integral gains :
Kc = 500;
Ic = 100;


% END OF USER DATA ENTRY SECTION


% INITIALISATION CODE

% half track
tfby2 = ftrack/2;
trby2 = rtrack/2;

% find passive axle weights, dimensions a and b, and total sprung mass
M = mf + mr;
wf = mf*g;
wr = mr*g;
a = mr*l/M;
b = mf*l/M;

% effective suspension stiffness :
Kseff = 1./(1./Ks + 1./Kts);

% anti-roll bar units to Nm/rad :
Karbf = stabf*180/pi; 	
Karbr = stabr*180/pi; 	

% weight offset in each suspension :
woff = [wf; wf; wr; wr]/2;

% complete initial conditions vector
% (match wheel speeds and forward speed)
x0 = zeros(10,1);
x0(1) = u0;
w0 = u0/rr*ones(4,1);

% parameters needed for vehicle BODY dynamics function
rc = [a, a, -b, -b;
    -tfby2, tfby2, -trby2, trby2
    0, 0, 0, 0];

I = [ixx, 0, -ixz; 0, iyy, 0; -ixz, 0, izz];
invI = inv(I);

disp(' ');
disp('6 dof parameters initialised');




