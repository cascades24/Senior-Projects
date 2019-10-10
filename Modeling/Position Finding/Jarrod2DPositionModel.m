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
A4 = [d3*sin(beta3)*cos(beta2)*cos(beta1) d3*cos(beta1)*sin(beta2)*cos(beta3)...
    d3*sin(beta2)*sin(beta3)];
A = [A1;A2;A3;A4];

%beaconRange = 100; %m
%beaconAngle = 200; %deg
%Beacon = [beaconRange*cosd(beaconAngle) beaconRange*sind(beaconAngle)];
Beacon = [45 45 45];

sb1 = norm(abs(Beacon-A1));
sb2 = norm(abs(Beacon-A2));
sb3 = norm(abs(Beacon-A3));
sb4 = norm(abs(Beacon-A4));

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
E4_1 = E(sb4,t);

%Sample next, right after?
E1_2 = E(sb1,t+1e-11);
E2_2 = E(sb2,t+1e-11);
E3_2 = E(sb3,t+1e-11);
E4_2 = E(sb4,t+1e-11);

%If meas 2>meas 1, use negative sign
% These are the phase of the wave at each antenna
phi_1 = sign(E1_2-E1_1)*-acos(E1_2/Emax);
phi_2 = sign(E2_2-E2_1)*-acos(E2_2/Emax);
phi_3 = sign(E3_2-E3_1)*-acos(E3_2/Emax);
phi_4 = sign(E4_2-E4_1)*-acos(E4_2/Emax);

%Following diagram:
phi12 = (phi_2-phi_1);
% if phi12>pi
%     phi12=2*pi-phi12; %Assume d1< lambda/2
% end
   
phi23 = (phi_3-phi_2);
% if phi23>pi
%     phi23=2*pi-phi23; %Assume d1< lambda/2
% end

x = dspec*phi12;
y = dspec*phi23;
a = dspec*(phi_4-phi_1);
b = dspec*(phi_4-phi_3);
c = dspec*(phi_4-phi_1);
d = dspec*(phi_4-phi_2);

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

gamma_1 = acos(a/d3);
gamma_2 = beta2+asin((abs(a)-abs(b))/d2)-pi/2;
if gamma_1 <= pi/2
    gamma = gamma_2;
else
    if gamma_2>0
        gamma = -gamma_1;
    else
        gamma = gamma_1;
    end
end

theta_1 = acos(c/d3);
theta_2 = beta3+asin((abs(c)-abs(d))/d1)-pi/2;
if theta_1 <= pi/2
    theta = theta_2;
else
    if theta_2>0
        theta = -theta_1;
    else
        theta = theta_1;
    end
end

pos = [sb1*sin(theta)*cos(alpha) sb1*sin(alpha) sb1*cos(theta)*cos(alpha)];

%% Plot it
figure
plot3(A(:,1)',A(:,2)',A(:,3)','b*','LineWidth',2);
hold on
grid on
grid minor
plot3(Beacon(1), Beacon(2), Beacon(3),'r*','LineWidth',2);
plot3(pos(1),pos(2),pos(3),'mx','LineWidth',2);
legend('Antenna Positions','Beacon Position','Computed Position')

%% Alright, Now Monte Carlo

