% Credits : Tim Bailey and Juan Nieto 2004. 

% In this programming assignment you have to perform EKF localization of a
% mobile robot traversing in a static environment with landmarks.
% You are given snippets of MATLAB files that help you in getting the task done.

% Note :(1) The configuration of the EKF-simulator is managed by the script file
    %   'configfile.m'. You have the flexibility to alter the parameters of
    %   the vehicle, sensors, etc.
    %   (2) There are also several switches that control certain filter
    %   options. You are free to choose them.
    %   (3) If you don't disturb this file the filter runs with the
    %   default parameters.

clc
% clear variables;
clear myfunction;
clear global;
close all;

% Loading the example_webmab.mat file. 
data = load('example_webmap.mat');

% This file has the following : 
% lm - set of landmarks (i.e the ground truth locations of landmarks)
% wp -  waypoints : These are specific locations in the environment that the robot is intended to pass through.
%       way points (these points help in generating controls to robot, you need not to worry about these wp's)

% lm = data.lm;
% temp_rand = [20,4,33,18,11,25,9,19,29,16,32,34,31,27,10,1,14,5,8,12];
% lm = lm(:,temp_rand);

% wp =data.wp;

% wp = load('matlab.mat');
% wp = wp.A';
global map;
map = 'map3.mat';

obs = load(map);
obs = obs.mdetails;
obs = [4*obs(4:end,1), 2*obs(4:end,2)-8*ones(size(obs,1)-3,1)]';

wp = A_Star();
wp(:,2) = wp(:,2) - 8*ones(size(wp,1),1);
wp = flipud(wp);
wp = [0 0; wp];
wp = wp';

% Before calling 'ekf_localization_sim', all you have to do is to code the prediction and the correction step of filter.
% for prediction : you have to fill up 'predict.m' file.
% for correction : you have to fill up 'update.m' file.

ekf_localization_sim(wp, obs);

% Enjoy by visualising the output :) 







