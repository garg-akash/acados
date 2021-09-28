addpath('/home/akash/Documents/PeterCorkeToolbox/common/')
addpath('/home/akash/Documents/PeterCorkeToolbox/rtb/')
addpath('/home/akash/Documents/PeterCorkeToolbox/smtb/')

% clear
close all

d1 =0.4; d2 = 0.39;d7=0.078;

%All link lengths and offsets are measured in m
clear L
%            theta    d           a       alpha
links = [
	    Link([0        0           0       pi/2])
		Link([0        0           0      -pi/2])
		Link([0        d1          0      -pi/2])
		Link([0        0           0       pi/2])
		Link([0        d2          0       pi/2])
		Link([0        0           0       -pi/2])
		Link([0        d7           0       0])
	];

LWR=SerialLink(links, 'name', 'Kuka LWR');

LWR.qlim(1,1) = -170*pi/180;
LWR.qlim(1,2) = 170*pi/180;
LWR.qlim(2,1) = -120*pi/180;
LWR.qlim(2,2) = 120*pi/180;
LWR.qlim(3,1) = -170*pi/180;
LWR.qlim(3,2) = 170*pi/180;
LWR.qlim(4,1) = -120*pi/180;
LWR.qlim(4,2) = 120*pi/180;
LWR.qlim(5,1) = -170*pi/180;
LWR.qlim(5,2) = 170*pi/180;
LWR.qlim(6,1) = -120*pi/180;
LWR.qlim(6,2) = 120*pi/180;
LWR.qlim(7,1) = -170*pi/180;
LWR.qlim(7,2) = 170*pi/180;

% LWR.jacob0([q_log(:,1)' 0])

LWR.teach

% q0 = q_log(:,1:end);%cat(1,q_ref,zeros(1,size(q_ref,2)));
% LWR.plot(q0')
% LWR.plot(q_output')

% q = LWR.ikcon(T0_we);