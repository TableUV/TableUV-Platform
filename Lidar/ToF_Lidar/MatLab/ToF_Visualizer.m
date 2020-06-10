clc;
close all;
clear all; 


data = importdata('ToFData6.txt');
topLeft_X             = data(:,1);
topLeft_Y             = data(:,2);
botRight_X            = data(:,3);
botRight_Y            = data(:,4);
RangeStatus           = data(:,5);
RangeMilliMeter       = data(:,6);
SignalRateRtnMegaCps  = data(:,7);
AmbientRateRtnMegaCps = data(:,8);
EffectiveSpadRtnCount = data(:,9);

x = topLeft_X ;
y = RangeMilliMeter;
z = botRight_Y;

plot3(x,y,z,'o')
xlabel('X')
ylabel('Y')
zlabel('Duration')
grid on



