%% Senior Projects Position Modeling Take 2
% Created: Anastasia Muszynski 10/7/19 based upon posModel by  Emily Webb 
% Modified: Anastasia Muszynski 10/7/19
% A model for finding position of beacon in 3 directions 
% based on posModel and information from Ambiguity Resolution in
% Interferometry (5 Antenna Direction finding)
%% Housekeeping!
clear all; close all; clc;

%% Initialize Position of Antennas and Beacon
A1 = [0 0 0];
A2 = [0 0 0.2];
A3 = [0 0.2 0];
A4 = [0.2 0 0];


% Spherical Coordinates Whoo!
psi = 45; % Azimuth angle, degrees
theta = 45; % Elvation angle, degrees
r = 200;    % Distance between receiver origin and transmitter origin (m)

% Center of Beacon
Bee = [r*sind(theta)*cosd(psi), r*sind(theta)*sind(psi), r*cosd(theta)];
%% Attitude Part: 
%Transmitters in same configuration as Recievers
T1 = A1; T2 = A2; T3 = A3; T4 = A4;
% Angles to rotate by (euler angle sequence ZYX)
eul = [pi/2 pi/3 pi/4]; % Convert to radians 
rotM= eul2rotm(eul);
T2 = rotM*T2'; 
T3 = rotM*T3'; 
T4 = rotM*T4'; 
rotm2eul([T4 T3 T2]);


% Translate to Beacon location 
T1 = T1 +Bee;
T2 = T2' +Bee; 
T3 = T3' +Bee;
T4 = T4' +Bee;

%% Testing Angle calculations (just position in 3d)

thetavec = linspace(0,179, 360);
psivec = linspace(0, 359, 360);

% Iterate and check elevation test cases
theta2 = zeros(length(thetavec));
psi2 = zeros(length(thetavec));
psi = 200;
    for i = 1:length(thetavec)
        theta = thetavec(i);
        Bee = [r*sind(theta)*cosd(psi), r*sind(theta)*sind(psi), r*cosd(theta)];
        [BeeCalc, theta2(i), psi2(i)]= test3D(A1, A2, A3,A4, Bee, 0, 0);
    end

% Iterate and check azimuth test cases    
theta3 = zeros( length(psivec));
psi3 = zeros( length(psivec));
    theta = 30;
    for i = 1:length(psivec)
        psi = psivec(i);
        Bee = [r*sind(theta)*cosd(psi), r*sind(theta)*sind(psi), r*cosd(theta)];
        [BeeCalc, theta3(i), psi3(i)]= test3D(A1, A2, A3, A4, Bee, 0, 0);
    end
    
 figure
   sgtitle({'Changing Elevation Angle Holding Azimuth Angle Constant at 200 degrees'})
   subplot(1,2,1)
   plot(thetavec, theta2)
   grid on 
   xlabel('Actual Beacon Theta Angle')
   ylabel('Calculated Beacon Theta Angle')
   subplot(1,2,2)
   plot(thetavec, psi2)
   grid on
   xlabel('Actual Beacon Psi Angle')
   ylabel('Calculated Beacon Psi Angle')
   
   figure
   sgtitle({'Changing Azimuth Angle Holding Elevation Angle Constant at 30 Degrees'})   
   subplot(1,2,1)
   plot(psivec, theta3)
   grid on 
   xlabel('Actual Beacon Theta Angle')
   ylabel('Calculated Beacon Theta Angle')
   subplot(1,2,2)
   plot(psivec, psi3)
   grid on
   xlabel('Actual Beacon Psi Angle')
   ylabel('Calculated Beacon Psi Angle')
  
   
 %% Testing Attitude  with phase errors
 
 N = 500; %Number of points
err = zeros(N,3); 
for i = 1:N
    [T1calc, theta1, psi1] = test3D(A1, A2, A3, A4, T1, .0001, 0);
    [T2calc, theta2, psi2] = test3D(A1, A2, A3, A4, T2, .0001, 0);
    [T3calc, theta3, psi3] = test3D(A1, A2, A3, A4, T3, .0001, 0);
    [T4calc, theta4, psi4] = test3D(A1, A2, A3, A4, T4, .0001, 0);

    [eul2] = findAttitude(T1calc, T2calc, T3calc, T4calc);
    err(i,:) = (eul2-eul)*180/pi;
end

fprintf('Euler Angle Error- Range and Phase Error\n')
MakePlots(err)
   
   %% Testing Attitude  with  range errors
 
 N = 500; %Number of points
err = zeros(N,3); 
for i = 1:N
    [T1calc, theta1, psi1] = test3D(A1, A2, A3, A4, T1, 0, 0.001);
    [T2calc, theta2, psi2] = test3D(A1, A2, A3, A4, T2, 0, 0.001);
    [T3calc, theta3, psi3] = test3D(A1, A2, A3, A4, T3, 0, 0.001);
    [T4calc, theta4, psi4] = test3D(A1, A2, A3, A4, T4, 0, 0.001);


    [eul2] = findAttitude(T1calc, T2calc, T3calc, T4calc);
    err(i,:) = (eul2-eul)*180/pi;
end
fprintf('Euler Angle Error- Phase Error\n')
 MakePlots(err)

   
      %% Testing Attitude  with  phase and range errors
 
 N = 500; %Number of points
err = zeros(N,3); 
for i = 1:N
    [T1calc, theta1, psi1] = test3D(A1, A2, A3, A4, T1, 0.0001, 0.001);
    [T2calc, theta2, psi2] = test3D(A1, A2, A3, A4, T2, 0.0001, 0.001);
    [T3calc, theta3, psi3] = test3D(A1, A2, A3, A4, T3, 0.0001, 0.001);
    [T4calc, theta4, psi4] = test3D(A1, A2, A3, A4, T4, 0.0001, 0.001);
    
    [eul2] = findAttitude(T1calc, T2calc, T3calc, T4calc);
    err(i,:) = (eul2-eul)*180/pi;
end
fprintf('Euler Angle Error- Range and Phase Error\n')
MakePlots(err)

%% Functions to Find Position and attitude
  
function [BeeCalc, theta, psi] = test3D(A1, A2, A3, A4, Bee, phaseErr, rangeErr)
% Inputs 3 reciever  and 1 beacon locations in cartesian coordinates, as
% well as variation term (for playing around with error, multiply by
% variable inside function to apply an error)

    % Separation distances between receiving antennas
    s12 = norm(abs(A1 - A2)); 
    s13 = norm(abs(A1 - A3));
    s14 = norm(abs(A1 - A4));

    % Find the ranges. (Assumed to be known)
    r1 = norm(abs(Bee-A1));
    r2 = norm(abs(Bee-A2));
    r3 = norm(abs(Bee-A3));
    r4 = norm(abs(Bee-A4));
    
    %% Initialize other Parameters
    f = 2.4e9; % Hz, frequency of sine wave
    c = 299792458; % m/s, the speed of light
    lambda = c/f; % m, wavelength
    dspec = lambda/(2*pi); % "specific distance"

    %% Actual Calculations
    r12 = r1-r2; % distance difference (m)
    r13 = r1-r3;
    r14 = r1-r4;
    
    % Error will only be applied if phaseErr is nonzero. phaseErr gives
    % order of magnitude for error.
    dphi12 = r12/dspec+normrnd(0,2)*phaseErr; % calculate phase difference between antennas 1 and 2
    dphi13 = r13/dspec+normrnd(0,2)*phaseErr; % calculate phase difference between antennas 1 and 3
    dphi14 = r14/dspec+normrnd(0,2)*phaseErr; % calculate phase difference between antennas 1 and 3
    % These values are what we will be measuring 
    
    % These are the calculations we will be doing to find  
    theta = acosd(lambda*dphi12/(2*pi*s12)); % Elevation 
    psi = atan2d(dphi13*s14,(dphi14*s13)); % Azimuth
    
    if psi <0 
      psi = 360+psi; 
    end 

    % Error will only be applied if rangeErr is nonzero. rangeErr gives
    % order of magnitude for error.
    r1 = r1+normrnd(0,2)*rangeErr;
    %% Finding Position, using A1 as reference antenna
    BeeCalc= [r1*sind(theta)*cosd(psi), r1*sind(theta)*sind(psi), r1*cosd(theta)];
   
end 

function [eul] = findAttitude(Tloc1, Tloc2, Tloc3, Tloc4)
    % Transmitter Coordinate Frame unit vectors
    Xt = Tloc4 - Tloc1; 
    Xt = Xt/norm(Xt);
    Yt = Tloc3 - Tloc1; 
    Yt = Yt/norm(Yt);
    Zt = Tloc2 - Tloc1; 
    Zt = Zt/norm(Zt);

    % Matrix describing transmitter attitude in receiver coordinate frame 
    T = [Xt' Yt' Zt'];
    
    % Finding Euler angles for comparison (Could also find quaternion
    % here)
    eul = rotm2eul(real(T)); 
    % We will want to calculate the quaternion, but that is a later us
    % problem. Comparing the error of the euler angles will be more intuitive
    % Matlab also has built in quaternion functions, but we will need to
    % write these if we're coding in C

end

function [] = MakePlots(err)
figure
   hold on 
   %Scatter plot of locations
   subplot(1,3,1)
   hold on 
   histogram(err(:,1))
   title('Error in Computation')
   xlabel('Angle 1 Error (degrees)')
   hold off
   
   subplot(1,3,2)
   %Histogram of errors'
   hold on 
   title('Error in Computation')
   histogram(err(:,2))
   xlabel('Angle 2 Error (degrees)')
   hold off 
   
    subplot(1,3,3)
   %Histogram of errors'
   hold on 
   title('Error in Computation')
   histogram(err(:,3))
   xlabel('Angle 3 Error (degrees)')
   hold off 
   
   hold off
   fprintf(' Angle 1: Mean: %d degrees, St.Dev: %d degrees\n', mean(err(:,1)), std(err(:,1)))
   fprintf(' Angle 2: Mean: %d degrees, St.Dev: %d degrees\n', mean(err(:,2)), std(err(:,2)))
   fprintf(' Angle 3: Mean: %d degrees, St.Dev: %d degrees\n', mean(err(:,3)), std(err(:,3)))
end 