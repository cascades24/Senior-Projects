%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RF Antenna Link Budget Calculator
% 
% This calculates the Link Margin for an RF Transmitter/Reciever
% Inputs are: Frequency, Bit energy to noise rati
% Design Margin, Data Rate, Noise Figure, Receiver Gain
% Transmitter Gain
%
% Author: Camilla Hallin
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Housekeeping
clear all
close all
clc

%%  Setup
D=[100,200,300,400]/1000;
for i=1:numel(D)
c=3e8;
f=2.5;
lamb= c/(f*1e9);
Boltzman=1.380e-23;
BER=10e-7;
EbNo=10; % Bit energy to noise ratio
Z=50e6; % Data Rate
PrNo=EbNo+10*log10(Z); % Carrier to Noise Ratio Density
DM=10; % Design Margin

MinPrNo=PrNo+DM;

RxTemp=8; % Kelvin for a 60 degree path 
RxLosses=-.5; % Typical Value
RxNF=2.5; % Can vary, this is based on LIME
RxF=290*10^(RxNF/10);
RxTs=RxTemp+((290*(1-0.89)/0.89))+((290*(RxF-1)/0.89));
RxNoisePower=10*log10(RxTs*Boltzman); %dBW-Hz

Gr=20; % Not sure what antenna gain to use

RxFOM=Gr/RxTs;


% Propagation Loss
Dist=D(i); %km
Ls=10*log10((lamb/(4*pi()*Dist*10^3))^2);
La=-5; % Atmospheric absorption
Lp=-.2; % Polarization Loss typical value


Gt=20; % Not sure what antenna gain to use
TxLosses=-.5;
TxWatts=linspace(.00001,.005,500);
TxPower=10*log10(TxWatts);
EIRP=TxPower+Gt;


RxPower=TxPower+Gt+Gr+Ls+La+Lp+RxLosses;
PrNoActual=RxPower-RxNoisePower;

TxNeed=MinPrNo-(Gt+Gr+Ls+La+Lp+RxLosses)+RxNoisePower;

LinkMargin=PrNoActual-MinPrNo;

%% Plot The Results
figure(1), plot(TxWatts*1000,LinkMargin,'LineWidth',1.1)
hold on
xlabel('Transmit Power (mW)')
ylabel('Link Margin (dB)')
title('Link Margin vs Transmit Power at 100 m','FontSize',14)
ltext{i}=strcat('D = ',num2str(round(1000*D(i))),' m');
end
plot(TxWatts*1000,zeros(1,numel(LinkMargin)),'g--','LineWidth',1.4)
legend(ltext,'Location','EastOutside')


%% Timing Accuracy Analysis
c=3e8;
dist=100; % Start with 100m 
data_rate=32e6; % 32 MHz processor (INPUT)
t_trav=dist/0.5/c; % "truth travel time"
t_start=3/data_rate; % Asuumed time to send start bits
t_buff=4/data_rate; % Assumed buffer time

packetSize=8; % 16 bit packet to decode
t_DM= packetSize*1/data_rate; % Assumed time to Demodulate/ recieve packet
ac=linspace(0,5,50); % Accuracy of clock (in fractions of a clock cycle)

% for j=1:50
% for i=1:1000
% t_startR=t_start+normrnd(0,1)*;
% t_DMR=t_DM+normrnd(0,1)*.13e-12;
% t_buffR=t_buff+normrnd(0,1)*.13e-12;
% 
% T_sum(i)=t_trav+2*t_startR+2*t_DMR+t_buffR;
% DistCal(i)=.5*c*(T_sum(i)-2*t_start-2*t_DMR-t_buff);
% 
% end
% var(j)=std(DistCal);
% end
% 
% figure(5), hist(DistCal)
% 
% figure(6)
% plot(ac./data_rate,var)
% title('Distance Accuracy vs Timing Accuracies')
% xlabel('Timing Accuracy')
% ylabel('1 \sigma error on Distance (m)')
% hold on


%% Averaging Benefit

%figure
TErr=linspace(1e-12,2/data_rate,200);
for j=1:10000
for i=1:numel(TErr)
t_startR=t_start+normrnd(0,1)*TErr(i);
t_DMR=t_DM+normrnd(0,1)*0*TErr(i);
t_buffR=t_buff+normrnd(0,1)*0*TErr(i);

T_sum2(i)=t_trav+2*t_startR+2*t_DMR+t_buffR;
DistCal2(i,j)=.5*c*(T_sum2(i)-2*t_start-2*t_DMR-t_buff);

end
if mod(j,10)==0
    avdata10(:,j/10)=mean(DistCal2(:,j/10:j/10+9),2);
end
if mod(j,20)==0
    avdata20(:,j/20)=mean(DistCal2(:,j/20:j/20+19),2);
end
if mod(j,30)==0
    avdata30(:,j/30)=mean(DistCal2(:,j/30:j/30+29),2);
end
if mod(j,100)==0
    avdata100(:,j/100)=mean(DistCal2(:,j/100:j/100+99),2);
end



% scatter(TErr,abs(DistCal2(:,j)),'b.')
%hold on
end
% hold on,scatter(1:numel(saveDist),saveDist),ylim([99.5, 100.5])
%% Standard Deviations

figure
plot(TErr,abs(mean(100-DistCal2,2))+abs(std(100-DistCal2,1,2)),'LineWidth',1.1)
hold on
plot(TErr,abs(mean(100-avdata10,2))+abs(std(100-avdata10,1,2)),'LineWidth',1.1)
plot(TErr,abs(mean(100-avdata20,2))+abs(std(100-avdata20,1,2)),'LineWidth',1.1)
plot(TErr,abs(mean(100-avdata30,2))+abs(std(100-avdata30,1,2)),'LineWidth',1.1)
plot(TErr,abs(mean(100-avdata100,2))+abs(std(100-avdata100,1,2)),'LineWidth',1.1)
plot(TErr,2*ones(1,numel(TErr)),'LineWidth',1.2)
title('Expected Distance Error vs Timing Error')
xlabel('Timing Error (s)')
ylabel('Average \mu Distance Error + 1 \sigma (m)')
legend('No Averaging','Averaging 10','Averaging 20','Averaging 30','Averaging 100')
ylim([0,10])

%% No averaging Histogram
index1=find(TErr>19e-9,1);
figure, hist(DistCal2(index1,1:1000))
title('Distance Estimate, Timing error of 20 ns')
xlabel('Distance Estimate (m)') 
ylabel('Frequency')
mean(DistCal2(index1,1:1000))
std(DistCal2(index1,1:1000))



%% Histogram For Averaging 10 with timing error ~20 ns
index10=find(TErr>19e-9,1);
figure, hist(avdata10(index10,:))
title('Distance with 10 Averaged, Timing error of 20 ns')
xlabel('Distance Estimate (m)') 
ylabel('Frequency')
mean(avdata10(index10,:))

%%
index30=find(TErr>31.3e-9,1);
figure, hist(avdata30(index30,1:333))
title('Distance with 30 Averaged, Timing error of 33 ns')
xlabel('Distance Estimate (m)') 
ylabel('Frequency')
mean(avdata30(index30,1:333))
std(avdata30(index30,1:333))
