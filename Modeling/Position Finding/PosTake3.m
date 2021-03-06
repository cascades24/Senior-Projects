%% Senior Projects Position Modeling Take 3
% Created: Brendan Lutes 10/8/19 based upon posModel by  Emily Webb anad
% Anastasia Muzinski
% Modified: Brendan Lutes 10/9/19
% A model for finding position of beacon in 3 directions 
% based on posModel and information from Ambiguity Resolution in
% Interferometry (5 Antenna Direction finding)
%% Housekeeping!
clear all; close all; clc;

%% Initialize Position of Antennas and Beacon
A1 = [0 -0.5 0]; %[m]
A2 = [0 0.5 0]; %[m]
A3 = [-0.5 0 0]; %[m]
A4 = [0.5 0 0]; %[m]
A5 = [0 0 0]; %[m]


% Spherical Coordinates Whoo!
alpha = 70; % Azimuth angle, degrees
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
err= test3D5A(A1, A2, A3, A4, A5, Bee, 1)


function [err] = test3D5A(A1, A2, A3, A4, A5, Bee, variation)
% Inputs 3 reciever  and 1 beacon locations in cartesian coordinates, as
% well as variation term (for playing around with error, multiply by
% variable inside function to apply an error)

    % Separation distances between receiving antennas
    s12 = norm(abs(A1 - A2)); 
    s13 = norm(abs(A1 - A3));
    s14 = norm(abs(A1 - A4));
    s15 = norm(abs(A1 - A5));
    s23 = norm(abs(A2 - A3));
    s24 = norm(abs(A2 - A4));
    s25 = norm(abs(A2 - A5));
    s34 = norm(abs(A3 - A4));
    s35 = norm(abs(A3 - A5));
    s45 = norm(abs(A4 - A5));
    

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
    r4 = norm(abs(Bee-A4));
    r5 = norm(abs(Bee-A5));
    

    %% Initialize other Parameters
    f = 10e6; % Hz, frequency of sine wave
    c = 299792458; % m/s, the speed of light
    lambda = c/f; % m, wavelength
    dspec = lambda/(2*pi); % "specific distance"

    %% Actual Calculations
    r12 = abs(r2-r1); % distance difference (m)
    r34 = abs(r4-r3); % 
 

    dphi12 = r12/dspec; % calculate phase difference between antennas 1 and 2
    dphi34 = r34/dspec;
    
    % These are the calculations we will be doing to find  
    
    theta = asind((dphi12*lambda/(2*pi*s12))^2+(dphi34*lambda/(2*pi*s34))^2);
    psi = atand(dphi12*s34/(dphi34*s12));
    
    
   

    

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