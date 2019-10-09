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

% Spherical Coordinates Whoo!
alpha = 45; % Azimuth angle, degrees
beta = 45; % Elvation angle, degrees
r = 200;    % Distance between receiver origin and transmitter origin (m)

% Center of Beacon
Bee = [r*sind(beta)*cosd(alpha), r*sind(beta)*sind(alpha), r*cosd(beta)];

%% Location of 3 transmitters with respect to beacon origin
% Add for attitude finding, TODO
% T1 = [0 0 0]+ Bee; 
% T2 = [0 0 0.05] + Bee; 
% T3 = [0 0.05 0] + Bee; 

% Transforming Beacon to different attitude 
% Creating transformation matrix

% Creating quaternion from transformation matrix
%% Running and plotting test case

%% Function to simulate converting recieved data to beacon location
err= test3D(A1, A2, A3, Bee, 1)


function [err] = test3D(A1, A2, A3, Bee, variation)
% Inputs 3 reciever  and 1 beacon locations in cartesian coordinates, as
% well as variation term (for playing around with error, multiply by
% variable inside function to apply an error)

    % Separation distances between receiving antennas
    s12 = norm(abs(A1 - A2)); 
    s13 = norm(abs(A1 - A3));

    % Quick Plot for sanity
    figure(1);
    plot3(A1(1),A1(2), A1(3),'*','MarkerSize',8);
    hold on
    plot3(A2(1),A2(2),A2(3),'*','MarkerSize',8);
    plot3(A3(1),A3(2), A3(3),'*','MarkerSize',8);
    plot3(Bee(1),Bee(2), Bee(3),'*','MarkerSize',8);
    grid on 
    grid minor
    % Sanity checked!


    % Find the ranges. (Assumed to be known)
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

    dphi12 = r12/dspec; % calculate phase difference between antennas 1 and 2
    dphi13 = r13/dspec; % calculate phase difference between antennas 1 and 3
    % These values are what we will be measuring 
    
    % These are the calculations we will be doing to find  
    theta = acosd(lambda*dphi12/(2*pi*s12)); % Elevation 
    psi = asind(dphi13*s13/(dphi12*s12)); % Azimuth
    

    %% Actual Position, using A1 as reference antenna
    Wasp = [r1*sind(theta)*cosd(psi), r1*sind(theta)*sind(psi), r1*cosd(theta)];

    % New plotting sanity check
    figure(2);
    set(gcf,'Color','White');
    plot3(Bee(1),Bee(2),Bee(3),'*','MarkerSize',8);
    hold on
    plot3(Wasp(1),Wasp(2),Wasp(3),'*','MarkerSize',8);
    legend('Actual Beacon Position','Predicted Beacon Position');
    grid on 
    grid minor
    %xlim([Bee(1)-1 Bee(1)+1]); ylim([Bee(2)-1 Bee(2)+1]);zlim([Bee(3)-1 Bee(3)+1]);
    % title('Actual and Predicted Beacon Position');

    % Absolute error 
    err = abs(Bee - Wasp);
    % Relative error
    %err = abs(Bee - Wasp)./abs(Bee);
    fprintf('The X direction error is: %.3f.\n',err(1));
    fprintf('The Y direction error is: %.3f.\n',err(2));
    fprintf('The Z direction error is: %.3f.\n',err(3));
    
    figure(1)
     plot3(Wasp(1),Wasp(2),Wasp(3),'*','MarkerSize',8);
    legend('Receiver 1', 'Receiver 2', 'Receiver 3','Actual Beacon Position','Predicted Beacon Position');
end 