%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ASEN 4018: Senior Design Phase Modelling Stuff 
%(Not using any range measurments)
% 
% Created: Jarrod Puseman 10/6/2019
% Modified: Anastasia Muszynski 10/7/2019
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Clean Up
clear; clc; close all;

%% Set Up Antennas and Beacon (Coordinates)
%For Drawing, see page 27 in Jarrod's notebook
d12 = .06; %m
d13 = .06; %m
beta1 = 90; %degrees

beeRange = 100; %m
beeAngle = 200; %deg
Bee = [beeRange*cosd(beeAngle) beeRange*sind(beeAngle)];
A1 = [0 0];
A2 = [d12 0];
A3 = [d13*cosd(beta1) d13*sind(beta1)];

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
    
%% Test all Theta values
thetavec = linspace(0,359, 360);
for i = 1:length(thetavec)
    Bee = [beeRange*cosd(thetavec(i)) beeRange*sind(thetavec(i))];
    alpha(i) = test2D(d12, d13, beta1, Bee, 0);
end
% figure()
% hold on
% plot(thetavec, alpha, 'bo')
% xlabel('Beacon Heading Angle (degrees)')
% ylabel('Beacon Heading Calculated Angle (degrees)')
% grid on 

%% Alright, Now Monte Carlo
N = 5000; %Number of points
alpha = zeros(N,1); 
Bee = [beeRange*cosd(beeAngle) beeRange*sind(beeAngle)];
for i = 1:N
    alpha(i) = test2D(d12, d13, beta1, Bee, 1);
end 
 figure
   histogram((alpha-beeAngle))
   grid on 
   xlabel('Beacon Heading Angle Error (degrees)')
   title({'Beacon Position Error for Normally Distributed Phase Angle Errors of 0.3 radians',...
       ' for Antenna Separation of 6 cm, Range of 100m, Heading Angle of 200 degrees'})
   fprintf('Mean: %d   Standard deviation: %d \n', mean(alpha-beeAngle), std(alpha-beeAngle))
   
   
  err = [beeRange*cosd(alpha), beeRange*sind(alpha)]-Bee;
     figure
    sgtitle({'Beacon Position Error for Normally Distributed Phase Angle Errors of 0.3 radians',...
       ' for Antenna Separation of 6 cm, Range of 100m, Heading Angle of 200 degrees'})
   subplot(1,2,1)
   histogram(err(:,1))
   grid on 
   xlabel('Beacon x-location error')
   subplot(1,2,2)
   histogram(err(:,2))
   grid on
   xlabel('Beacon y-location error')

%% Trying again, with sample averaging 
n = 10; % samples to average 
alpha = zeros(N,n);
for i = 1:N
    for j = 1:n
        alpha(i,j) = test2D(d12, d13, beta1, Bee, 1);
    end 
end 
alpha = mean(alpha,2);
 figure
   histogram((alpha-beeAngle))
   grid on 
   xlabel('Beacon Heading Angle Error (degrees)')
   title({'Beacon Position Error for Normally Distributed Phase Angle Errors of 0.3 radians',...
       ' for Antenna Separation of 6 cm, Range of 100m, Heading Angle of 200 degrees',...
       'Averaging 10 Samples'})   
   fprintf('Averaging 10 Samples: Mean: %d   Standard deviation: %d \n', mean(alpha-beeAngle), std(alpha-beeAngle))

   %% Trying again, with sample averaging 
n = 30; % samples to average 
alpha = zeros(N,n);
for i = 1:N
    for j = 1:n
        alpha(i,j) = test2D(d12, d13, beta1, Bee, 1);
    end 
end 
alpha = mean(alpha,2);
 figure
   histogram((alpha-beeAngle))
   grid on 
   xlabel('Beacon Heading Angle Error (degrees)')
      title({'Beacon Position Error for Normally Distributed Phase Angle Errors of 0.3 radians',...
       ' for Antenna Separation of 6 cm, Range of 100m, Heading Angle of 200 degrees',...
       'Averaging 30 Samples'})      
   fprintf('Averaging 30 Samples: Mean: %d   Standard deviation: %d \n', mean(alpha-beeAngle), std(alpha-beeAngle))


%% Function
function [alpha] = test2D(d12, d13, beta1, Bee, useError)
    
    A1 = [0 0];
    A2 = [d12 0];
    A3 = [d13*cosd(beta1) d13*sind(beta1)];

    sb1 = norm(abs(Bee-A1));
    sb2 = norm(abs(Bee-A2));
    sb3 = norm(abs(Bee-A3));

    %% Set Up RF Parameters
    f = 2.4e9; % Hz, frequency of sine wave
    c = 299792458; % m/s, the speed of light
    lambda = c/f; % m, wavelength
    dspec = lambda/(2*pi); % "specific distance"
    
    r12 = sb1-sb2; % distance difference (m)
    r13 = sb1-sb3; % distance difference (m)

    phi12 = r12/dspec; % calculate phase difference 
    phi13 = r13/dspec; % calculate phase difference 
    
    if useError ==1 
        phi12 = phi12+normrnd(0,2)*0.1;
        phi13 = phi13+normrnd(0,2)*0.1;
    end 
    
    alpha = atan2d(phi13*d12,(phi12*d13));
    if alpha <0 
      alpha = 360+alpha; 
    end 
end
