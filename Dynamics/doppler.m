% Analyze Doppler Effect

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ASEN 4028: Senior Projects
% Final Oral Review
% Jarrod Puseman
% Dr. Jackson
% Created:  4/8/2020
% Modified: 4/8/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Phase 3
c = 299704644.54;
f = 3.3e9;
lambda = c/f;
v = 6.7056; %m/s
a =@(t) v*t-50;
b =@(t) v*t -40;
vrel =@(t) (a(t)*v + b(t)*v)./(sqrt(a(t).^2 + b(t).^2));
time = 0:100;
velc = vrel(time);
%plot(time,velc)
vrelmax = max(abs(velc)); %hardcode

fmax = (c+vrelmax)/(c)*f;
fmin = (c-vrelmax)/(c)*f;
lambdanew1 = c/fmin;
lambdanew2 = c/fmax;
