clc;
close all;
clear all; 

data = importdata('ToFData.txt');
topLeft_X   = data(:,1);
topLeft_Y   = data(:,2);
botRight_X  = data(:,3);
botRight_Y  = data(:,4);
RangeStatus = data(:,5);