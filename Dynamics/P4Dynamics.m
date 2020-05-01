% Experiment with some path dynamics for Senior Projects

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ASEN 4028: Senior Projects
% Created by Jarrod Puseman
% Created:  3/11/2020
% Modified: 3/11/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc;close all;

% Consider the Relative position of sensor and beacon moving in circles
% at different speeds
%%%%%%%% Tuning Knobs %%%%%%%%
v1 = 4.4704; %m/s
v2 = 6.7056; %m/s
r = 25; %m
tmax = 75; %s
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Explicit Positions
t=linspace(0,tmax,500);
theta1 = v1*t./r;
theta2 = v2*t./r;
x1 = 35+r*cos(theta1);
x2 = -35-r*cos(theta2);
y1 = r*sin(theta1);
y2 = r*sin(theta2);

%Relative Position of track 2 to track 1
xrel = x2-x1;
yrel = y2-y1;

%Relative Position in Rotating Frame
% x out nose, y to right, z down
xrelbod = xrel.*sin(theta1) + yrel.*cos(theta1);
yrelbod = xrel.*cos(theta1) + yrel.*sin(theta1);

plotData(x1,y1,x2,y2,xrel,yrel,xrelbod,yrelbod,'P4');
%M = makeMovie(t,x1,y1,x2,y2,xrel,yrel,xrelbod,yrelbod,'P4mov');

