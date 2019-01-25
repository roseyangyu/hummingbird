%% Modify the IP addresses of your ROS enabled robots/simulators here
% Copyright 2017 The MathWorks, Inc.

% Paths
addpath('utility');
addpath('test');

% Connect and Startup
connectRobot;
% auto_takeoff;

% Mininum Rotation and Translation for an apriltag
minRot = 30;
minTrans = 1;

% Sample Time
sampleTime = 0.01;
    
% Mininum Tracking  
minTrackRot = 10;
minTrackTrans = 0.2;

% TF Tree
tftree = rostf;
