%%% Configuration file
%%% Permits various adjustments to parameters of the filter.
 
% Control Parameters


global a;
global b;
global Xt;
global Xi;

Xt = [100, 100];
Xi = [0, 0];
a = 1.6;
b = 3.2;
t = 0:0.0250:8;
path = [100*sin(t'), 4*t'.^2.*cos(t')];

V=3; % Velocity of the robot in 'meters/second'.
MAXG= 30*pi/180;  % Maximum steering angle (orientation) of robot in 'radians' (-MAXG < g < MAXG)
RATEG= 20*pi/180; % Maximum rate of change of steer angle in 'radians/second'
WHEELBASE=  4;     % Robot wheel-base in 'meters'
DT_CONTROLS=0.025;% Time interval between control signals in 'seconds'

% Noise in execution of control signals

sigmaV=  0.1*2;         % standard deviation of velocity in 'meters/second'
sigmaG= (0.05*10*pi/180); % standard deviation of orientation in 'radians'
 
Q = [sigmaV^2 0; 0 sigmaG^2]; % Covariance matrix
 
 
% Observation Parameters

MAX_RANGE= 30.0;           % Maximum range of the laser range finder in 'metres'
DT_OBSERVE=8*DT_CONTROLS; % Time interval between observations in 'seconds'

% NOise in range finder measurements / observations.

sigmaX = 0.1*2;          % standard deviation of X measuments in 'metres'
sigmaY = 0.1*2;          % standard deviation of Y measuments in 'metres'
sigmaB = (0.1*10*pi/180); % standard deviation of IMU angle measuments in 'radians'
R= [sigmaX^2 0 0; 0 sigmaY^2 0; 0 0 sigmaB^2];

% waypoint proximity
AT_WAYPOINT= 0.02; % Minimum distance treshold to switch to next waypoint.
NUMBER_LOOPS= 2; % number of loops through the waypoint list

% switches
SWITCH_CONTROL_NOISE= 1; % if 0, velocity and orientation are perfect
SWITCH_SENSOR_NOISE = 1; % if 0, measurements are perfect
SWITCH_SEED_RANDOM= 0; % if not 0, seed the randn() with its value at beginning of simulation (for repeatability)
 
