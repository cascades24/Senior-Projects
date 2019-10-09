%% Senior Projects Position Modeling Take 2
% Created: Emily Webb National Garlic Lovers Day
% Modified: Anastasia Muszynski 10/7/19

%% Housekeeping!
clear all; close all; clc;

%% Initialize Position of Antennas and Beacon
A1 = [0 0]; 
A3 = [0 0.5];
Bee = [100*cosd(35) 100*sind(35)];

%% Testing Different Antenna Separations (holding A1 constant, moving A2), no random error

N =50;    % Number of samples
err1 = zeros(N,2);  % Preallocate space fo vector of error values
thetaerr1 = zeros(N,1);  % Preallocate space fo vector of error values
% Currently changing x location of A2 
A2vec(:,2) = linspace(0.05, 0.5, N);

% Iterating through A2 values
for i = 1:N
    [err1(i,:), thetaerr1(i,:)] = test2D(A1,A2vec(i,:), Bee, 0);
end 
% New plotting sanity check
    figure
    set(gcf,'Color','White');
    hold on
    plot(A2vec(:,2), abs(err1(:,1)) ,'b.','MarkerSize',8);
    plot(A2vec(:,2), abs(err1(:,2)) ,'r.','MarkerSize',8)
    legend('Error in X Position','Error in Y Position');
    xlabel('Separation Distance of Reciever A2 from A1 (m)')
    ylabel('Absolute Position Error (m)')
    grid on 
    title('Error in Beacon Position Based on Separation');

%% Testing Different Antenna Separations, Monte Carlo Simulation

N =5000;    % Number of samples
err1 = zeros(N,2);  % Preallocate space fo vector of error values
thetaerr1 = zeros(N,1);  % Preallocate space fo vector of error values
A2 = [0 0.06];

for i = 1:N
    [err1(i,:), thetaerr1(i,:)] = test2D(A1, A2, Bee,1);
end 
% Histograms of x and y error
   figure
   sgtitle({'Beacon Position Error for Normally Distributed Range Errors of 1mm',...
       ' for Antenna Separation of 6 cm, Range of 100m, Heading Angle of 35 degrees'})
   subplot(1,2,1)
   grid on
   histogram(err1(:,1))
   xlabel('X-Position Error (m)')
   subplot(1,2,2)
   grid on 
   histogram(err1(:,2))
   xlabel('Y-Position Error (m)')
   
   figure
   histogram(thetaerr1(:,1))
   grid on 
   xlabel('Beacon Heading Angle Error (degrees)')
   title({'Beacon Position Error for Normally Distributed Range Errors of 1mm',...
       ' for Antenna Separation of 6 cm, Range of 100m, Heading Angle of 35 degrees'})
   
% Reset A2 location 
A2 = [0 0.2];
%% Testing different ranges 
N = 500;
rvec = linspace(10,1000,N);
err2 = zeros(N,2);  % Preallocate space fo vector of error values
thetaerr2 = zeros(N,1);  % Preallocate space fo vector of error values

for i = 1:N
    Bee = [rvec(i)*cosd(35) rvec(i)*sind(35)];
    [err2(i,:), thetaerr2(i,:)] = test2D(A1, A2, Bee, 0);
end 

% New plotting sanity check
    figure
    set(gcf,'Color','White');
    hold on
        plot(rvec, abs(err2(:,1)) ,'b.');
        plot(rvec, abs(err2(:,2)) ,'r.');
    legend('Error in X Position','Error in Y Position');
    xlabel('Distance of Reciever from Beacon (m)')
    ylabel('Absolute Position Error (m)')
    grid on 
    title('Error in Beacon Position Based on Beacon/ Transmitter Separation');
%% Testing Different Ranges, Monte Carlo Simulation

N =5000;    % Number of samples
err2 = zeros(N,2);  % Preallocate space fo vector of error values
thetaerr2 = zeros(N,1);  % Preallocate space fo vector of error values

r1 = 100; 
Bee = [r1*cosd(35) r1*sind(35)];

for i = 1:N
    [err2(i,:), thetaerr2(i,:)] = test2D(A1, A2, Bee,1);
end 
% Histograms of x and y error
   figure
   sgtitle({'Beacon Position Error for Normally Distributed Ranging Errors of 1mm',...
       ' for Antenna Separation of 20 cm, Range of 100m, Heading Angle of 35 degrees'})
   subplot(1,2,1)
   histogram(err2(:,1))
   grid on 
   xlabel('X-Position Error (m)')
   subplot(1,2,2)
   histogram(err2(:,2))
   grid on 
   xlabel('X-Position Error (m)')
   
   figure
   histogram(thetaerr2(:,1))
   grid on 
   xlabel('Beacon Heading Angle Error (degrees)')
   title({'Beacon Position Error for Normally Distributed Range Errors of 1mm',...
       ' for Antenna Separation of 20 cm, Range of 100m, Heading Angle of 35 degrees'})
      fprintf('Mean: %d   Standard deviation: %d', mean(err3), std(err3))
    
    %% Testing different theta values 
N = 50;
r = 100;
thetavec = linspace(0, 88,N);
err3 = zeros(N,2);  % Preallocate space fo vector of error values
thetaerr3 = zeros(N,1);  % Preallocate space fo vector of error values

for i = 1:N
    Bee = [r*cosd(thetavec(i)) r*sind(thetavec(i))];
    [err3(i,:), thetaerr3(i,:)] = test2D(A1, A2, Bee, 0);
end 
% New plotting sanity check
    figure
    set(gcf,'Color','White');
    hold on
    plot(thetavec, err3(:,1) ,'b.');
    plot(thetavec, err3(:,2) ,'r.');
     %ylim([-.1, .2])
    legend('Error in X Position','Error in Y Position');
    xlabel('Heading Angle (from X axis) (degrees)')
    ylabel('Absolute Position Error (m)')
    grid on 
    title('Error in Beacon Position Based on Theta Angle');
 
%% Testing Different Ranges, Monte Carlo Simulation

N =5000;    % Number of samples
err3 = zeros(N,2);  % Preallocate space fo vector of error values
thetaerr3 = zeros(N,1);  % Preallocate space fo vector of error values
theta = 10;
Bee = [r1*cosd(theta) r1*sind(theta)];

for i = 1:N
    [err3(i,:), thetaerr3(i,:)] = test2D(A1, A2, Bee,1);
end 
% Histograms of x and y error
   figure
    sgtitle({'Beacon Position Error for Normally Distributed Range Errors of 1mm',...
       ' for Antenna Separation of 20m, Range of 100m, Heading Angle of 10 degrees'})
   subplot(1,2,1)
   histogram(err3(:,1))
   grid on 
   xlabel('Beacon x-location error')
   subplot(1,2,2)
   histogram(err3(:,2))
   grid on
   xlabel('Beacon y-location error')
    
   figure
   histogram(thetaerr3(:,1))
   grid on 
   xlabel('Beacon Heading Angle Error (degrees)')
   title({'Beacon Position Error for Normally Distributed Range Errors of 1mm',...
       ' for Antenna Separation of 6 cm, Range of 100m, Heading Angle of 10 degrees'})
   fprintf('Mean: %d   Standard deviation: %d', mean(err3), std(err3))
   
   
   
function [err, thetaErr] = test2D(A1, A2, Bee, useError)
    % Separation distances between receiving antennas
    s12 = norm(abs(A1 - A2)); 
    
    % Applying Systematic Error to r1 and r2 measurements
    r1 = norm(abs(Bee-A1));
    r2 = norm(abs(Bee-A2)); 
    
    % Applying Systematic Error to r1 and r2 measurements, if useError flag
    % set to 1. Otherwise, assume no normal error
    
    if useError == 1 
        r1 = r1+normrnd(0,2)*0.001; % Normally distributed error (0.001 = 1mm error in range measurements)
        r2 = r2+normrnd(0,2)*0.001; % Normally distributed error
    end 

    %% Initialize other Parameters
    f = 2.4e9; % Hz, frequency of sine wave
    c = 299792458; % m/s, the speed of light
    lambda = c/f; % m, wavelength
    dspec = lambda/(2*pi); % "specific distance"

    %% Actual Calculations
    r12 = abs(r1-r2); % distance difference (m)

    dphi12 = r12/dspec; % calculate phase difference 

    theta12 = asind(lambda*dphi12/(2*pi*s12)); % calculate heading angle

    %% Actual Position, using A1 as reference antenna
    Wasp = real([r1*cosd(theta12) r1*sind(theta12)]);

    % New plotting sanity check
    % figure(2);
    % set(gcf,'Color','White');
    % plot(Bee(1),Bee(2),'*','MarkerSize',8);
    % hold on
    % plot(Wasp(1),Wasp(2),'*','MarkerSize',8);
    % legend('Actual Beacon Position','Predicted Beacon Position');
    % xlim([Bee(1)-1 Bee(1)+1]); ylim([Bee(2)-1 Bee(2)+1]);
    % title('Actual and Predicted Beacon Position');

    % Absolute position error
    err = Wasp-Bee;
    
    % Absolute heading error
    theta = asind(Bee(2)/norm(Bee));
    thetaErr = real(theta12 -theta);
    % Relative error
    %err = abs(Bee - Wasp)./abs(Bee);
%     fprintf('The X direction error is: %.3f.\n',err(1));
%     fprintf('The Y direction error is: %.3f.\n',err(2));
end 
