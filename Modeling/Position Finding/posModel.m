%% Senior Projects Position Modeling Take 2
% Created: National Garlic Lovers Day
% Modified: National Noodle Day

%% Housekeeping!
clear; close all; clc;

%% Initialize Position of Antennas and Beacon
A1 = [0 0]; 
A2 = [0 0.2];
A3 = [0 0.5];

% Distances between antennas
s12 = norm(abs(A1 - A2)); 
s13 = norm(abs(A1 - A3));

Bee = [100*cosd(35) 100*sind(35)];

% Quick Plot for sanity
% figure(1);
% plot(A1(1),A1(2),'*','MarkerSize',8);
% hold on
% plot(A2(1),A2(2),'*','MarkerSize',8);
% plot(A3(1),A3(2),'*','MarkerSize',8);
% plot(Bee(1),Bee(2),'*','MarkerSize',8);
% Sanity checked!

% Find the ranges.
r1 = norm(abs(Bee-A1));
r2 = norm(abs(Bee-A2));
r3 = norm(abs(Bee-A3));

%% Initialize other Parameters
f = 2.4e9; % Hz, frequency of sine wave
c = 299792458; % m/s, the speed of light
lambda = c/f; % m, wavelength
dspec = lambda/(2*pi); % "specific distance"

%% Actual Calculations

r12 = abs(r1-r2);
r13 = abs(r1-r3);

dphi12 = r12/dspec;

theta12 = asind(lambda*dphi12/(2*pi*s12));

