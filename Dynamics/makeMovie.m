function [M] = makeMovie(t,x1,y1,x2,y2,xrel,yrel,xrelbod,yrelbod,name)
% makeMovie makes a movie of the data spit into it for Senior Projects
% Format of call: plotData(t,x1,y1,x2,y2,xrel,yrel,xrelbod,yrelbod,name)
% Returns: figure handle

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ASEN 4028: Senior Projects
% Created by Jarrod Puseman
% Created:  3/11/2020
% Modified: 3/11/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Make a movie
len = length(t);
fig=figure;

M = struct('cdata', cell(1,len), 'colormap', cell(1,len));
for i = 1:len
    %Setup
    subplot(1,3,1)
    plot(x1(1:i),y1(1:i),'b.','LineWidth',2,'Markersize',10)
    hold on
    plot(x2(1:i),y2(1:i),'r.','LineWidth',2,'Markersize',10)
    legend('Sensor', 'Beacon')
    grid on
    grid minor
    title('Newtonian Paths')
    xlabel('x Position [m]')
    xlim([-70 70])
    ylabel('y Position [m]')
    ylim([-70 70])
    axis square

    subplot(1,3,2)
    plot(xrel(1:i),yrel(1:i),'.','Linewidth',2,'Markersize',10)
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
    plot(xrelbod(1:i),yrelbod(1:i),'.','Linewidth',2,'Markersize',10)
    hold on
    grid on
    grid minor
    title('Relative Position in Rotating Body Frame')
    xlabel('Relative x Position [m]')
    xlim([-100 100])
    ylabel('Relative y Position [m]')
    ylim([-125 75])
    axis square

    suptitle('Phase 4 Nonlinear Dynamics')
    set(gcf, 'Position', [100, 100, 1100, 450]) %Reposition
    
    M(i) = getframe(fig);
end
%Make a movie
close all
%movie(M);
%implay(M)
v = VideoWriter(name,'Archival');
open(v)
writeVideo(v,M)
close(v)
end


    