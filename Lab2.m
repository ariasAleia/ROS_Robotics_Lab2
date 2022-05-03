%% Lab 2 - Matlab and ROS with PhantomX
clear
clc
l = [14.5, 10.7, 10.7, 9]; % Length of links

L(1) = Link('revolute','alpha',pi/2,'a',0,   'd',l(1),'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(2) = Link('revolute','alpha',0,   'a',l(2),'d',0,   'offset',pi/2,'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,   'a',l(3),'d',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,   'a',0,   'd',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
PhantomX = SerialLink(L,'name','Px');

%Tool orientation following NAO Convention
PhantomX.tool = [0 0 1 l(4); -1 0 0 0; 0 -1 0 0; 0 0 0 1];

%Plot PhantomX
PhantomX.plot([0 0 0 0], 'notiles', 'noname');
hold on
trplot(eye(4),'rgb','arrow','length',15,'frame','0')
ws = [-50 50];
axis([repmat(ws,1,2) 0 60])
PhantomX.teach()

%% MTHs PhantomX
syms q1 q2 q3 q4 as real
%MTHs
T1_0 = L(1).A(q1);
%Sim
T1_0(1,2) = 0;
T1_0(2,2) = 0;
T1_0(3,3) = 0;
T2_1 = L(2).A(q2)
T3_2 = L(3).A(q3)
T4_3 = L(4).A(q4)
%% MTH From base to TCP
MTH_TCP_base = simplify(T1_0*T2_1*T3_2*T4_3*PhantomX.tool)
%% Showing different configurations
% First configuration
qt1 = deg2rad([0 90 -30 90]);
PhantomX.plot(qt1, 'notiles', 'noname');
%% Second configuration
qt2 = deg2rad([0 -60 30 45]);
PhantomX.plot(qt2, 'notiles', 'noname');
%% Third configuration
qt3 = deg2rad([85 -45 0 -30]);
PhantomX.plot(qt3, 'notiles', 'noname');
