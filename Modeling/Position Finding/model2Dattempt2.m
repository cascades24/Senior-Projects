%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ASEN 4018: Senior Design Phase Modelling Stuff 
%(Not using any range measurments)
% 
% Created: Jarrod Puseman 10/6/2019
% Modified: Anastasia Muszynski 10/13/2019
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Clean Up
clear; clc; close all;

%% Set Up Antennas and Beacon (Coordinates)
%For Drawing, see page 27 in Jarrod's notebook
d12 = 0.061225; %m
d13 = 0.061225; %m
beta1 = 90; %degrees

beeRange = 100; %m
beeAngle = 200; %deg
Bee = [beeRange*cosd(beeAngle) beeRange*sind(beeAngle)];
A1 = [0 0];
A2 = [d12 0];
A3 = [d13*cosd(beta1) d13*sind(beta1)];

% Plotting Test case
 figure
    hold on
    plot(A1(1), A1(2) ,'*','MarkerSize',8);
    plot(A2(1), A2(2) ,'*','MarkerSize',8);
    plot(A3(1), A3(2) ,'*','MarkerSize',8);
    plot(Bee(1), Bee(2) ,'*','MarkerSize',8);
    legend('Receiver1','Receiver2','Receiver3','Bee Actual');
    xlabel('X (m)')
    ylabel('Y (m)')
    grid on 
    title('Beacon and Reciever Locations');
    
%% Test all Theta values (make sure we are calculating all of them correctly)
thetavec = linspace(0,359, 360);
for i = 1:length(thetavec)
    Bee = [beeRange*cosd(thetavec(i)) beeRange*sind(thetavec(i))];
    theta(i) = test2D(d12, d13, beta1, Bee, 0,0);
end
figure()
hold on
plot(thetavec, theta, 'bo')
xlabel('Beacon Heading Angle (degrees)')
ylabel('Beacon Heading Calculated Angle (degrees)')
grid on 
hold off

%% Alright, Now Monte Carlo
N = 500; %Number of points
theta = zeros(N,1); 
BeeCalc = zeros(N,2); 
Bee = [beeRange*cosd(beeAngle) beeRange*sind(beeAngle)];
for i = 1:N
   [theta(i), BeeCalc(i,:)]  = test2D(d12, d13, beta1, Bee, 0,1);
end 
 
% generate Histogram of angles, scatter plot of computed positions, and
% scatterplot of range error, print mean and standard deviation info
fprintf('No Averaging\n')
 MakePlots(theta,BeeCalc, Bee, beeAngle, beeRange)
 
%% Trying again, with sample averaging 
n = 20; % samples to average 
theta = zeros(N,n);
BeeCalc = zeros(N,n,2); 
for i = 1:N
    for j = 1:n
        [theta(i,j), BeeCalc(i,j,:)] = test2D(d12, d13, beta1, Bee, .08,2);
    end 
end 
theta = mean(theta,2);
BeeCalc = mean(BeeCalc,2);
 % generate Histogram of angles, scatter plot of computed positions, and
% scatterplot of range error
fprintf('Averaging %i pts\n',n)
 MakePlots(theta, BeeCalc,Bee, beeAngle, beeRange)
 
   %% Trying again, with more sample averaging 
n = 50; % samples to average 
theta = zeros(N,n);
BeeCalc = zeros(N,n,2); 
for i = 1:N
    for j = 1:n
        [theta(i,j), BeeCalc(i,j,:)] = test2D(d12, d13, beta1, Bee, .13, 2);
    end 
end 
theta = mean(theta,2);
BeeCalc = mean(BeeCalc,2);
 % generate Histogram of angles, scatter plot of computed positions, and
% scatterplot of range error
fprintf('Averaging %i pts\n',n)
 MakePlots(theta,BeeCalc, Bee, beeAngle, beeRange)
   
%% Function
function [theta, BeeCalc] = test2D(d12, d13, beta1, Bee, phaseErr, rangeErr)
    
    A1 = [0 0];
    A2 = [d12 0];
    A3 = [d13*cosd(beta1) d13*sind(beta1)];

    r1 = norm(abs(Bee-A1));
    r2 = norm(abs(Bee-A2));
    r3 = norm(abs(Bee-A3));

    %% Set Up RF Parameters
    f = 2.4e9; % Hz, frequency of sine wave
    c = 299792458; % m/s, the speed of light
    lambda = c/f; % m, wavelength
    dspec = lambda/(2*pi); % "specific distance"
    
    r12 = r1-r2; % distance difference (m)
    r13 = r1-r3; % distance difference (m)

    % Note: error will only be applied if phaseErr is nonzero
    phi12 = r12/dspec+normrnd(0,2)*phaseErr; % calculate phase difference 
    phi13 = r13/dspec+normrnd(0,2)*phaseErr; % calculate phase difference 
    
    
    theta = atan2d(phi13*d12,(phi12*d13));
    if theta <0 
      theta = 360+theta; 
    end 
    
    % Note: error will only be applied if rangeErr is nonzero
    r1 = r1+normrnd(0,2)*rangeErr;
    BeeCalc = [r1*cosd(theta), r1*sind(theta)];
    
end


function []= MakePlots(theta, BeeCalc, Bee, beeAngle, beeRange) 
figure
   histogram((theta-beeAngle),'FaceColor', [0, 0, 1], 'EdgeColor','k')
   grid on 
   xlabel('Beacon Heading Angle Error (degrees)')
   title({'Beacon Heading Angle Error'})
   fprintf('Mean: %d degrees  Standard deviation: %d degrees \n', mean(theta-beeAngle), std(theta-beeAngle))
   % Find beacon location based upon heading
    % Calculate total distance error
    err = zeros(length(BeeCalc), 1);
    for i=1:length(BeeCalc)
        err(i) = norm(Bee-BeeCalc(i,:));
    end
   % Calculating percent of cases within error bound of 2%
   bound = .02*beeRange;
   percent = length(find(err<bound))/length(err) * 100;

   
   figure
   hold on 
   %Scatter plot of locations
   subplot(1,2,1)
   hold on 
   scatter(BeeCalc(:,1),BeeCalc(:,2),'k*')
   scatter(Bee(1), Bee(2),'r*')
   xlabel('X-Position (m)')
   ylabel('Y-Position (m)')
   grid on
   legend('Computed Positions', 'Actual Beacon')
   title('Computed Position')
   hold off
   
   subplot(1,2,2)
   %Histogram of errors'
   hold on 
   title('Error in Computation')
   histogram(err,'FaceColor', [0, 0, 1], 'EdgeColor','k')
   xlabel('Beacon Location error (m)')
   hold off 
   hold off
   % Print Mean and standard deviation of errors, and percent acceptable
   % cases
   fprintf('Range Error: \n Mean: %d m, St.Dev: %d m\n Percent of errors within 2m, %d \n\n', mean(err), std(err), percent)
end
