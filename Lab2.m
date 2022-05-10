%% Lab 2 - Matlab and ROS with PhantomX
clear
clc

l = [14.5, 10.63, 10.65, 8.97]; % Length of links
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


%% Communication with ROS
rosinit; %Connecting with master node


%% Different configurations in Matlab with ROS and Dynamixel

q1 = [deg2rad([90 0 0 0]) 0];
q2 = [deg2rad([-20 20 -20 20]) 0];
q3 = [deg2rad([30 -30 30 -30]) 0];
q4 = [deg2rad([-90 15 -55 17]) 0];
q5 = [deg2rad([-90 45 -55 45]) 0];

%% With q1
PhantomX.plot(q1(1:4), 'notiles', 'noname');
moveRobot(q1)
%% With q2
PhantomX.plot(q2(1:4), 'notiles', 'noname');
moveRobot(q2)
%% With q3
PhantomX.plot(q3(1:4), 'notiles', 'noname');
moveRobot(q3)
%% With q4
PhantomX.plot(q4(1:4), 'notiles', 'noname');
moveRobot(q4)
%% With q5
PhantomX.plot(q5(1:4), 'notiles', 'noname');
moveRobot(q5)

%% Topic subscription

jointSub = rossubscriber('/dynamixel_workbench/joint_states', 'DataFormat','struct'); % We create the subscriptor
[msgSub,status,statustext] = receive(jointSub,10); % We receive the message

disp("Angle in radians for each joint:")
disp(" ")

for i = 1:5
    disp("Joint" + i + ": " + msgSub.Position(i))
end

%% Shutdown rosnode
rosshutdown;
%%

%Function to move joints and gripper
function output = moveRobot(q)
    offsetID = 0;
    motorSvcClient = rossvcclient('/dynamixel_workbench/dynamixel_command'); %Creation of client for the service
    motorCommandMsg = rosmessage(motorSvcClient); %Creation of the service message
    

    %Torque adjustment
    motorCommandMsg.AddrName = "Torque_Limit";
    torque = [600, 400, 400, 400, 400];
    for i= 1: length(q) 
        motorCommandMsg.Id = i+offsetID;
        motorCommandMsg.Value = torque(i);
        call(motorSvcClient,motorCommandMsg);
    end

    %Movement of joints    
    motorCommandMsg.AddrName = "Goal_Position";
    for i= 1: length(q) %To move joints
        disp(i)
        motorCommandMsg.Id = i+offsetID;
        disp(round(mapfun(rad2deg(q(i)),-150,150,0,1023)))
        motorCommandMsg.Value = round(mapfun(rad2deg(q(i)),-150,150,0,1023));
        call(motorSvcClient,motorCommandMsg); 
        pause(1);
    end
    
end

