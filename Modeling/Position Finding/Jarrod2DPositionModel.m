%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ASEN 4018: Senior Design Phase Modelling Stuff 
%(Not using any range measurments)
% 
% Created:  10/6/2019
% Modified: 10/7/2019
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Clean Up
clear; clc; close all;

%% Set Up Antennas and Beacon (Coordinates)
%For Drawing, see page 27 in Jarrod's notebook
d1 = .2; %m
d2 = .2; %m
d3 = .2; %m
beta1 = 90*pi/180; %radians
beta2 = pi/2;
beta3 = pi/2;

A1 = [0 0 0];
A2 = [d1 0 0];
A3 = [d2*cos(beta1) d2*sin(beta1) 0];

beaconRange = 100; %m
beaconAngle = 200; %deg
Beacon = [beaconRange*cosd(beaconAngle) beaconRange*sind(beaconAngle)];

sb1 = norm(abs(Beacon-A1));
sb2 = norm(abs(Beacon-A2));
sb3 = norm(abs(Beacon-A3));

%% Set Up RF Parameters
f = 10e6; % Hz, frequency of sine wave
c = 299792458; % m/s, the speed of light
lambda = c/f; % m, wavelength
dspec = lambda/(2*pi); % "specific distance"


%% Set Up RF Transmission
Emax = 1;
phi_0 = 0;
k = 2*pi/lambda;
w = 2*pi*f;
%Assume Simultaneous Sampling
% Magnetic Field as Function of Distance
E =@(x,t) Emax*cos(k*x-w*t+phi_0); 

%% Now we Direction-Find
% Assume E-max is known.
% Assume two measurements are performed in
% succession to get the phase in [0,360]

%Sample intitially - t=1 sec
t = 0; %sec
E1_1 = E(sb1,t); 
E2_1 = E(sb2,t); 
E3_1 = E(sb3,t); 

%Sample next, right after?
E1_2 = E(sb1,t+1e-11);
E2_2 = E(sb2,t+1e-11);
E3_2 = E(sb3,t+1e-11);

%If meas 2>meas 1, use negative sign
% These are the phase of the wave at each antenna
phi_1 = sign(E1_2-E1_1)*-acos(E1_2/Emax);
phi_2 = sign(E2_2-E2_1)*-acos(E2_2/Emax);
phi_3 = sign(E3_2-E3_1)*-acos(E3_2/Emax);

%Following diagram:
phi12 = (phi_2-phi_1);
phi23 = (phi_3-phi_2);

x = dspec*phi12;
y = dspec*phi23;

% Compute Direction Using Trig
alpha_1 = acos(x/d1);
alpha_2 = beta1+asin((abs(x)-abs(y))/d2)-pi/2;
if alpha_1 <= pi/2
    alpha = alpha_2;
else
    if alpha_2>0
        alpha = -alpha_1;
    else
        alpha = alpha_1;
    end
end

%% Alright, Now Monte Carlo

