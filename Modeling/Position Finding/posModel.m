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
r12 = abs(r1-r2); % distance difference (m)
r13 = abs(r1-r3);

dphi12 = r12/dspec; % calculate phase difference 

theta12 = asind(lambda*dphi12/(2*pi*s12));

%% Actual Position, using A1 as reference antenna
Wasp = [r1*cosd(theta12) r1*sind(theta12)];

% New plotting sanity check
figure(2);
set(gcf,'Color','White');
plot(Bee(1),Bee(2),'*','MarkerSize',8);
hold on
plot(Wasp(1),Wasp(2),'*','MarkerSize',8);
legend('Actual Beacon Position','Predicted Beacon Position');
xlim([Bee(1)-1 Bee(1)+1]); ylim([Bee(2)-1 Bee(2)+1]);
title('Actual and Predicted Beacon Position');

err = abs(Bee - Wasp);
fprintf('The X direction error is: %.3f.\n',err(1));
fprintf('The Y direction error is: %.3f.\n',err(2));

