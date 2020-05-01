function fig = plotData(x1,y1,x2,y2,xrel,yrel,xrelbod,yrelbod,name)
% plotData makes a plot of the data spit into it for Senior Projects
% Format of call: plotData(x1,y1,x2,y2,xrel,yrel,xrelbod,yrelbod,name,movie)
% Returns: figure handle

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ASEN 4028: Senior Projects
% Created by Jarrod Puseman
% Created:  3/11/2020
% Modified: 3/11/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fig = figure;
subplot(1,3,1)
plot(x1,y1,'b','LineWidth',2)
hold on
plot(x2,y2,'r','LineWidth',2)
grid on
grid minor
title('Newtonian Paths')
xlabel('x Position [m]')
xlim([-70 70])
ylabel('y Position [m]')
ylim([-70 70])
axis square
legend('Sensor', 'Beacon')

subplot(1,3,2)
plot(xrel,yrel,'Linewidth',2)
hold on
grid on
grid minor
title('Relative Position in Global Frame')
xlabel('Relative x Position [m]')
xlim([-125 -25])
ylabel('Relative y Position [m]')
ylim([-50 50])
axis square

subplot(1,3,3)
plot(xrelbod,yrelbod,'Linewidth',2)
hold on
grid on
grid minor
title('Relative Position in Rotating Body Frame')
xlabel('Relative x Position [m]')
xlim([-100 100])
ylabel('Relative y Position [m]')
ylim([-125 75])
axis square

suptitle('Phase 4 Test Information')
set(gcf, 'Position', [100, 100, 1100, 450]) %Reposition
print(name,'-dpng')
end